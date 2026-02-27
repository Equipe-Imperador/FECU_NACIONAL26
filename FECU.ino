#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include "wit_c_sdk.h"
#include "REG.h"

// --- 1. CONFIGURAÇÃO DE TAXAS (em Milissegundos) ---
#define TAXA_IMU_MS        14   // ~70Hz (Leitura Wit-Motion e Envio SD)
#define TAXA_ENVIO_CAN_MS  14   // ~70Hz (Acompanha a taxa do IMU)
#define TAXA_ANALOG_MS     200   // 5Hz  (Freio, Pressões)
#define TAXA_VEL_MS        20   // 50Hz  (Velocidade)

// --- 2. Estrutura de Dados e Sincronismo ---
struct DadosDinamica {
    uint32_t timestamp;
    float pedalFreio;
    float presDiant, presCM;
    float estercamento, correnteDif;
    bool acionamentoDif; 
    float v_LF, v_RF;
    float acc[3], gyro[3], angle[3];
};

DadosDinamica estadoAtual;
SemaphoreHandle_t xMutexEstado;
QueueHandle_t filaSD;
File dataFile;
char nomeArquivo[20];

// SDK Wit
extern "C" { extern int16_t sReg[REGSIZE]; }
static volatile char s_cDataUpdate = 0;

// --- 3. Definições de Pinos ---
#define PIN_PEDAL_F    36
#define PIN_PRES_D     35
#define PIN_PRES_CM    32
#define PIN_STEER      33
#define PIN_CUR_DIF    34
#define PIN_AC_DIF     26   
#define PIN_SPD_LF     25
#define PIN_SPD_RF      4

#define JY901_RX 17
#define JY901_TX 16

#define CAN_CS 15
SPIClass SPI_CAN(HSPI);
MCP_CAN CAN0(CAN_CS);

#define SD_CS 5

volatile unsigned long deltaLF = 0, deltaRF = 0;
volatile unsigned long lastLF = 0, lastRF = 0;

// --- 4. ISRs Velocidade ---
void IRAM_ATTR isrLF() {
    unsigned long agora = micros();
    if (agora - lastLF > 1000) { deltaLF = agora - lastLF; lastLF = agora; }
}
void IRAM_ATTR isrRF() {
    unsigned long agora = micros();
    if (agora - lastRF > 1000) { deltaRF = agora - lastRF; lastRF = agora; }
}

// --- 5. Helpers Wit-Motion ---
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) { Serial1.write(p_data, uiSize); }
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    for(int i = 0; i < uiRegNum; i++) {
        if(uiReg == AZ) s_cDataUpdate |= 0x01;
        if(uiReg == GZ) s_cDataUpdate |= 0x02;
        if(uiReg == Yaw) s_cDataUpdate |= 0x04;
        uiReg++;
    }
}
static void Delayms(uint16_t ucMs) { vTaskDelay(pdMS_TO_TICKS(ucMs)); }

// --- 6. Protótipos das Tasks e Funções ---
void vTaskIMU(void *pvParameters);
void vTaskAnalogicos(void *pvParameters);
void vTaskVelocidade(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskEnvioCAN(void *pvParameters);

float lerPressaoMPa(int pino);
void enviarMsgCAN(uint32_t id, float valor);

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
    pinMode(PIN_SPD_LF, INPUT_PULLUP);
    pinMode(PIN_SPD_RF, INPUT_PULLUP);
    pinMode(PIN_AC_DIF, INPUT_PULLUP); 
    
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_LF), isrLF, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_RF), isrRF, FALLING);

    Serial1.begin(921600, SERIAL_8N1, JY901_RX, JY901_TX);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);

    SPI_CAN.begin(14, 12, 13, CAN_CS);
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { delay(500); }
    CAN0.setMode(MCP_NORMAL);

    if (SD.begin(SD_CS)) {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/FECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;pedF;presD;presCM;LF;RF;accX;accY;accZ;dif_atv");
            dataFile.flush();
        }
    }

    xMutexEstado = xSemaphoreCreateMutex();
    filaSD = xQueueCreate(100, sizeof(DadosDinamica));

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);

    xTaskCreatePinnedToCore(vTaskIMU,        "IMU",   4096, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskAnalogicos, "Ana",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskVelocidade, "Vel",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD,         "SD",    4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnvioCAN,   "CAN",   4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// -------------------------------------------------------------------
// TAREFA 1: IMU Wit-Motion (~70Hz) -> MESTRE DO SD
// -------------------------------------------------------------------
void vTaskIMU(void *pvParameters) {
    for (;;) {
        while (Serial1.available()) {
            WitSerialDataIn(Serial1.read());
        }

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.timestamp = millis();

        if (s_cDataUpdate) {
            estadoAtual.acc[0] = sReg[AX] / 32768.0f * 16.0f;
            estadoAtual.acc[1] = sReg[AY] / 32768.0f * 16.0f;
            estadoAtual.acc[2] = sReg[AZ] / 32768.0f * 16.0f;
            s_cDataUpdate = 0;
        }

        // Como essa é a tarefa mais rápida, ela envia para o SD para não perder dados
        xQueueSend(filaSD, &estadoAtual, 0);
        xSemaphoreGive(xMutexEstado);
        
        vTaskDelay(pdMS_TO_TICKS(TAXA_IMU_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 2: ANALÓGICOS E PEDAIS (50Hz)
// -------------------------------------------------------------------
void vTaskAnalogicos(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.pedalFreio = analogReadMilliVolts(PIN_PEDAL_F);
        estadoAtual.estercamento = analogReadMilliVolts(PIN_STEER);
        estadoAtual.correnteDif = analogReadMilliVolts(PIN_CUR_DIF);
        estadoAtual.acionamentoDif = (digitalRead(PIN_AC_DIF) == LOW); 
        estadoAtual.presDiant = lerPressaoMPa(PIN_PRES_D);
        estadoAtual.presCM = lerPressaoMPa(PIN_PRES_CM);
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_ANALOG_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 3: VELOCIDADE DAS RODAS (50Hz)
// -------------------------------------------------------------------
void vTaskVelocidade(void *pvParameters) {
    for (;;) {
        noInterrupts();
        unsigned long dLF = deltaLF; 
        unsigned long dRF = deltaRF;
        interrupts();

        float v_lf_calc = (dLF > 500000) ? 0 : (1.70f * 3.6f) / (dLF / 1000000.0f);
        float v_rf_calc = (dRF > 500000) ? 0 : (1.70f * 3.6f) / (dRF / 1000000.0f);

        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.v_LF = v_lf_calc;
        estadoAtual.v_RF = v_rf_calc;
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_VEL_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 4: SD DATALOGGER
// -------------------------------------------------------------------
void vTaskSD(void *pvParameters) {
    DadosDinamica s;
    int ct = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &s, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%.1f;%.2f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%d\n", 
                    s.timestamp, s.pedalFreio, s.presDiant, 
                    s.presCM, s.v_LF, s.v_RF, s.acc[0], s.acc[1], s.acc[2], s.acionamentoDif);
                
                // Grava fisicamente no cartão a cada ~71 linhas (1 segundo em 70Hz)
                if (++ct >= 71) { 
                    dataFile.flush(); 
                    ct = 0; 
                }
            }
        }
    }
}

// -------------------------------------------------------------------
// TAREFA 5: ENVIO CAN (Core 1 - ~70Hz)
// -------------------------------------------------------------------
void vTaskEnvioCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        DadosDinamica p = estadoAtual;
        xSemaphoreGive(xMutexEstado);
            
        enviarMsgCAN(0x400, p.pedalFreio);
        enviarMsgCAN(0x402, p.presDiant);
        enviarMsgCAN(0x403, p.presCM);
        enviarMsgCAN(0x404, p.acc[0]); //X
        enviarMsgCAN(0x405, p.acc[1]); //Y
		enviarMsgCAN(0x406, p.acc[2]); //Z
		enviarMsgCAN(0x407, (float)p.acionamentoDif);
        enviarMsgCAN(0x203, p.v_LF);
        enviarMsgCAN(0x204, p.v_RF);

        vTaskDelay(pdMS_TO_TICKS(TAXA_ENVIO_CAN_MS));
    }
}

// -------------------------------------------------------------------
// FUNÇÕES AUXILIARES
// -------------------------------------------------------------------
float lerPressaoMPa(int pino) {
    float v = (analogReadMilliVolts(pino) / 1000.0f) * 1.5f;
    float mpa = ((v - 0.5f) * 400.0f) / 145.0f;
    return (mpa < 0) ? 0.0f : mpa;
}

void enviarMsgCAN(uint32_t id, float valor) {
    int16_t val = (int16_t)(valor * 100.0f);
    byte b[2] = {(byte)(val >> 8), (byte)(val & 0xFF)};
    CAN0.sendMsgBuf(id, 0, 2, b);
}
