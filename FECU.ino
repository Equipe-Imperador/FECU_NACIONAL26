/*
 * PROJETO: TELEMETRIA RECU - TRANSMISSOR 2 (FECU - BRAKE / STEER / DIFF / ACC)
 * VERSÃO: 5.3 (Acelerômetro Z adicionado no ID 0x409)
 */

#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include "wit_c_sdk.h"
#include "REG.h"

// --- 1. Estruturas e Globais ---
struct DadosDinamica {
    uint32_t timestamp;
    float pedalFreio;
    float presDiant, presCM;
    float estercamento, correnteDif;
    bool acionamentoDif; 
    float v_LF, v_RF;
    float acc[3], gyro[3], angle[3];
};

QueueHandle_t filaCAN, filaSD;
File dataFile;
char nomeArquivo[20];

// SDK Wit
extern "C" { extern int16_t sReg[REGSIZE]; }
static volatile char s_cDataUpdate = 0;

// --- 2. Definições de Pinos ---
#define PIN_PEDAL_F    36   // SENSOR_VP
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

// Variáveis de Interrupção para Velocidade
volatile unsigned long deltaLF = 0, deltaRF = 0;
volatile unsigned long lastLF = 0, lastRF = 0;

// --- 3. ISRs Velocidade ---
void IRAM_ATTR isrLF() {
    unsigned long agora = micros();
    if (agora - lastLF > 1000) { deltaLF = agora - lastLF; lastLF = agora; }
}
void IRAM_ATTR isrRF() {
    unsigned long agora = micros();
    if (agora - lastRF > 1000) { deltaRF = agora - lastRF; lastRF = agora; }
}

// --- 4. Helpers Wit-Motion ---
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

// --- Protótipos ---
void vTaskProcessa(void *pvParameters);
void vTaskEnvio(void *pvParameters);
void vTaskSD(void *pvParameters);

// -------------------------------------------------------------------
void setup() {
    Serial.begin(921600);
    
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
            // Cabeçalho atualizado com Y e Z
            dataFile.println("ms;pedF;presD;presCM;LF;RF;accX;accY;accZ;dif_atv");
            dataFile.flush();
        }
    }

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);

    filaCAN = xQueueCreate(10, sizeof(DadosDinamica));
    filaSD = xQueueCreate(30, sizeof(DadosDinamica));

    xTaskCreatePinnedToCore(vTaskProcessa, "Proc", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskSD, "SD", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskEnvio, "CAN", 4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// --- TAREFA PROCESSAMENTO (Core 0) ---
void vTaskProcessa(void *pvParameters) {
    DadosDinamica d;
    for (;;) {
        while (Serial1.available()) WitSerialDataIn(Serial1.read());

        if (s_cDataUpdate) {
            d.timestamp = millis();
            
            // Acelerômetro (X, Y e Z)
            d.acc[0] = sReg[AX] / 32768.0f * 16.0f;
            d.acc[1] = sReg[AY] / 32768.0f * 16.0f;
            d.acc[2] = sReg[AZ] / 32768.0f * 16.0f;

            // Analógicos e Digital Dif
            d.pedalFreio = analogReadMilliVolts(PIN_PEDAL_F);
            d.estercamento = analogReadMilliVolts(PIN_STEER);
            d.correnteDif = analogReadMilliVolts(PIN_CUR_DIF);
            d.acionamentoDif = (digitalRead(PIN_AC_DIF) == LOW); 

            // Pressões (MPa)
            auto calcMPa = [](int p) {
                float v = (analogReadMilliVolts(p) / 1000.0f) * 1.5f;
                float mpa = ((v - 0.5f) * 400.0f) / 145.0f;
                return (mpa < 0) ? 0.0f : mpa;
            };
            d.presDiant = calcMPa(PIN_PRES_D);
            d.presCM = calcMPa(PIN_PRES_CM);

            // Velocidades Dianteiras
            noInterrupts();
            unsigned long dLF = deltaLF; unsigned long dRF = deltaRF;
            interrupts();
            d.v_LF = (dLF > 500000) ? 0 : (1.70f * 3.6f) / (dLF / 1000000.0f);
            d.v_RF = (dRF > 500000) ? 0 : (1.70f * 3.6f) / (dRF / 1000000.0f);

            xQueueSend(filaCAN, &d, 0);
            xQueueSend(filaSD, &d, 0);
            s_cDataUpdate = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- TAREFA ENVIO CAN (Core 1) ---
void vTaskEnvio(void *pvParameters) {
    DadosDinamica p;
    esp_task_wdt_add(NULL);
    for (;;) {
        if (xQueueReceive(filaCAN, &p, portMAX_DELAY)) {
            esp_task_wdt_reset();
            
            auto env = [](uint32_t id, float v) {
                int16_t val = (int16_t)(v * 100.0f);
                byte b[2] = {(byte)(val >> 8), (byte)(val & 0xFF)};
                CAN0.sendMsgBuf(id, 0, 2, b);
            };

            env(0x400, p.pedalFreio);
            env(0x402, p.presDiant);
            env(0x403, p.presCM);
            env(0x404, p.acc[0]); // Acc X
            env(0x405, p.acc[1]); // Acc Y
            env(0x406, p.v_LF);
            env(0x407, p.v_RF);
            env(0x408, (float)p.acionamentoDif);
            env(0x409, p.acc[2]); // <--- NOVO: Acc Z no ID 0x409
        }
    }
}

// --- TAREFA SD (Core 0) ---
void vTaskSD(void *pvParameters) {
    DadosDinamica s;
    int ct = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &s, portMAX_DELAY)) {
            if (dataFile) {
                // Adicionado acc[1] e acc[2] no log SD
                dataFile.printf("%u;%.1f;%.2f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%d\n", 
                    s.timestamp, s.pedalFreio, s.presDiant, 
                    s.presCM, s.v_LF, s.v_RF, s.acc[0], s.acc[1], s.acc[2], s.acionamentoDif);
                if (++ct >= 50) { dataFile.flush(); ct = 0; }
            }
        }
    }
}
