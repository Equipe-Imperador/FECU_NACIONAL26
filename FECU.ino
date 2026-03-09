#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include "wit_c_sdk.h"
#include "REG.h"

// --- 1. CONFIGURAÇÃO DE TAXAS (em Milissegundos) ---
#define TAXA_IMU_MS        14   // ~70Hz 
#define TAXA_REDE_CAN_MS   10   // Mais rápido para não perder comandos e enviar telemetria (~100Hz max delay)
#define TAXA_ANALOG_MS     200  // 5Hz  
#define TAXA_VEL_MS        20   // 50Hz 

// --- 2. CONFIGURAÇÃO MECÂNICA E TIMEOUTS ---
#define DENTES_DIANT       5
#define DIAMETRO_DIANTEIRO 0.56f
#define PI_VAL             3.1415926535f
const float CIRC_DIANT = PI_VAL * DIAMETRO_DIANTEIRO;

const unsigned long DEBOUNCE_VEL_DIANT = 10000; 
const unsigned long TIMEOUT_US         = 800000; 

// --- 3. Estrutura de Dados e Sincronismo ---
struct DadosDinamica {
    uint32_t timestamp;
    float pedalFreio;
    float presDiant, presCM;
    float estercamento;
    bool correnteDif; // Agora é Bool lido do analogico para enviar na CAN
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

// --- 4. Definições de Pinos ---
#define PIN_PEDAL_F    36
#define PIN_PRES_D     35
#define PIN_PRES_CM    32
#define PIN_STEER      33
#define PIN_CUR_DIF    34      // Leitura analógica da corrente
#define PIN_SPD_LF     25
#define PIN_SPD_RF     4

// Pinos de Acionamento Físico (Relés)
#define PIN_AC_DIF     26   
#define PIN_AC_BUZINA  27 // Defina o pino da buzina na FECU   

#define JY901_RX 16
#define JY901_TX 17

#define CAN_CS 15
SPIClass SPI_CAN(HSPI);
MCP_CAN CAN0(CAN_CS);

#define SD_CS 5

// --- Variáveis Voláteis (ISRs Velocidade) ---
volatile unsigned long deltaLF = 0, lastLF = 0, anteriorLF = 0;
volatile unsigned long deltaRF = 0, lastRF = 0, anteriorRF = 0;

// --- 5. ISRs Velocidade ---
void IRAM_ATTR isrLF() {
    unsigned long t = micros(); 
    unsigned long d = t - anteriorLF;
    if (d > DEBOUNCE_VEL_DIANT) { 
        deltaLF = d; 
        anteriorLF = t; 
        lastLF = t; 
    }
}

void IRAM_ATTR isrRF() {
    unsigned long t = micros(); 
    unsigned long d = t - anteriorRF;
    if (d > DEBOUNCE_VEL_DIANT) { 
        deltaRF = d; 
        anteriorRF = t; 
        lastRF = t; 
    }
}

// --- 6. Helpers Wit-Motion ---
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

// --- 7. Protótipos das Tasks e Funções ---
void vTaskIMU(void *pvParameters);
void vTaskAnalogicos(void *pvParameters);
void vTaskVelocidade(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskRedeCAN(void *pvParameters); // Renomeada

float lerPressaoMPa(int pino);
void enviarMsgCAN(uint32_t id, float valor);

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
    pinMode(PIN_SPD_LF, INPUT_PULLUP);
    pinMode(PIN_SPD_RF, INPUT_PULLUP);
    
    // Configura Relés como saída e inicia desligados
    pinMode(PIN_AC_DIF, OUTPUT);
    digitalWrite(PIN_AC_DIF, LOW);
    pinMode(PIN_AC_BUZINA, OUTPUT);
    digitalWrite(PIN_AC_BUZINA, LOW);
    
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_LF), isrLF, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_RF), isrRF, RISING);

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
    esp_task_wdt_deinit();
    esp_task_wdt_init(&twdt_config);

    xTaskCreatePinnedToCore(vTaskIMU,        "IMU",   4096, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskAnalogicos, "Ana",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskVelocidade, "Vel",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD,         "SD",    4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskRedeCAN,    "CAN",   4096, NULL, 2, NULL, 1);
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

        xQueueSend(filaSD, &estadoAtual, 0);
        xSemaphoreGive(xMutexEstado);
        
        vTaskDelay(pdMS_TO_TICKS(TAXA_IMU_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 2: ANALÓGICOS E PEDAIS (5Hz)
// -------------------------------------------------------------------
void vTaskAnalogicos(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.pedalFreio = analogReadMilliVolts(PIN_PEDAL_F);
        estadoAtual.estercamento = analogReadMilliVolts(PIN_STEER);
        
        // Interpreta a tensão do sensor de corrente como booleano (tem corrente ou não)
        // Ajuste o limiar (ex: > 500mV = ativo) de acordo com o seu sensor
        int valorCorrente = analogReadMilliVolts(PIN_CUR_DIF);
        estadoAtual.correnteDif = (valorCorrente > 500); 
        
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
        unsigned long agora = micros();
        
        noInterrupts();
        unsigned long sDLF = deltaLF; unsigned long sLLF = lastLF;
        unsigned long sDRF = deltaRF; unsigned long sLRF = lastRF;
        interrupts();

        float v_lf_calc = 0, v_rf_calc = 0;

        if (agora - sLLF < TIMEOUT_US && sDLF > 0) {
            v_lf_calc = (CIRC_DIANT * 3600000.0f) / (float(sDLF) * DENTES_DIANT);
        }
        if (agora - sLRF < TIMEOUT_US && sDRF > 0) {
            v_rf_calc = (CIRC_DIANT * 3600000.0f) / (float(sDRF) * DENTES_DIANT);
        }

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
                    s.presCM, s.v_LF, s.v_RF, s.acc[0], s.acc[1], s.acc[2], s.correnteDif);
                
                if (++ct >= 71) { 
                    dataFile.flush(); 
                    ct = 0; 
                }
            }
        }
    }
}

// -------------------------------------------------------------------
// TAREFA 5: REDE CAN (Envia Dados e Recebe Comandos)
// -------------------------------------------------------------------
void vTaskRedeCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t tempoUltimoEnvio = 0;

    for (;;) {
        esp_task_wdt_reset();
        
        // 1. ESCUTA COMANDOS DA MECU
        if (CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            if (len == 2) {
                int16_t valorInt = (rxBuf[0] << 8) | rxBuf[1];
                
                if (rxId == 0x500) { // Comando Diferencial
                    digitalWrite(PIN_AC_DIF, (valorInt > 0) ? HIGH : LOW);
                } 
                else if (rxId == 0x501) { // Comando Buzina
                    digitalWrite(PIN_AC_BUZINA, (valorInt > 0) ? HIGH : LOW);
                }
            }
        }

        // 2. ENVIA TELEMETRIA (~70Hz)
        if (millis() - tempoUltimoEnvio >= 14) { 
            xSemaphoreTake(xMutexEstado, portMAX_DELAY);
            DadosDinamica p = estadoAtual;
            xSemaphoreGive(xMutexEstado);
                
            enviarMsgCAN(0x400, p.pedalFreio);
            enviarMsgCAN(0x402, p.presDiant);
            enviarMsgCAN(0x403, p.presCM);
            enviarMsgCAN(0x404, p.acc[0]); 
            enviarMsgCAN(0x405, p.acc[1]); 
            enviarMsgCAN(0x406, p.acc[2]); 
            enviarMsgCAN(0x407, (float)p.correnteDif); // Reporta se atracou com sucesso
            enviarMsgCAN(0x203, p.v_LF);
            enviarMsgCAN(0x204, p.v_RF);
            
            tempoUltimoEnvio = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(TAXA_REDE_CAN_MS));
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
