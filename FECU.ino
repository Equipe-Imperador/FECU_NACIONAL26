#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include "wit_c_sdk.h"
#include "REG.h"

// --- 1. CONFIGURAÇÃO DE TAXAS (em Milissegundos) ---
#define TAXA_IMU_MS        14   // ~70Hz (Mestre de telemetria)
#define TAXA_REDE_CAN_MS   14   // Sincronizado com a IMU (~70Hz)
#define TAXA_ANALOG_MS     50   // 20Hz 
#define TAXA_VEL_MS        20   // 50Hz

// --- Média Móvel ---
#define TAMANHO_FILTRO 5        // Quantidade de amostras para suavização da velocidade

// --- Calibração do Pedal de Freio (Sensor Hall - Lógica Invertida) ---
#define PEDAL_SOLTO_MV       2800  // mV com pedal solto (0%)
#define PEDAL_PRESSIONADO_MV 1780  // mV com pedal no fundo (100%)

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
    float pedalFreio;     // Em Porcentagem (0.00 a 100.00%) com Curva Quadrática
    float presDiant, presCM;
    float estercamento;   
    bool correnteDif; 
    float v_LF, v_RF;
    float acc[3], gyro[3], angle[3]; // Dados da IMU
};

DadosDinamica estadoAtual;
SemaphoreHandle_t xMutexEstado;
QueueHandle_t filaSD;
File dataFile;
char nomeArquivo[30];

// --- Variáveis do SDK Wit-Motion ---
extern "C" { extern int16_t sReg[REGSIZE]; }
static volatile char s_cDataUpdate = 0;

// --- 4. Definições de Pinos e Hardware ---
#define PIN_PEDAL_F    36
#define PIN_PRES_D     35
#define PIN_PRES_CM    32
#define PIN_STEER      33
#define PIN_CUR_DIF    34      
#define PIN_SPD_LF     25
#define PIN_SPD_RF     4

// Pinos de Acionamento Físico (Relés)
#define PIN_AC_DIF     26   
#define PIN_AC_BUZINA  27 

// UART da IMU JY901
#define JY901_RX 16
#define JY901_TX 17

// Pinos CAN (Barramento 1)
#define CAN_CS   15
#define CAN_SCK  14
#define CAN_MISO 12
#define CAN_MOSI 13

// Pinos SD (Barramento 2 isolado)
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// Instâncias SPI e CAN
MCP_CAN CAN0(CAN_CS);        
SPIClass sdSPI(HSPI);        

// --- Variáveis Voláteis (ISRs Velocidade) ---
volatile unsigned long deltaLF = 0, lastLF = 0, anteriorLF = 0;
volatile unsigned long deltaRF = 0, lastRF = 0, anteriorRF = 0;

// --- 5. ISRs Velocidade ---
void IRAM_ATTR isrLF() {
    unsigned long t = micros(); 
    unsigned long d = t - anteriorLF;
    if (d > DEBOUNCE_VEL_DIANT) { 
        deltaLF = d; anteriorLF = t; lastLF = t; 
    }
}

void IRAM_ATTR isrRF() {
    unsigned long t = micros(); 
    unsigned long d = t - anteriorRF;
    if (d > DEBOUNCE_VEL_DIANT) { 
        deltaRF = d; anteriorRF = t; lastRF = t; 
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
void vTaskRedeCAN(void *pvParameters); 

float lerPressaoMPa(int pino);
void enviarMsgCAN(uint32_t id, float valor);

// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
    Serial.begin(921600);
    Serial.println("\n[FECU] INICIANDO - MODO DUAL SPI BUS + IMU OTIMIZADA");

    pinMode(PIN_SPD_LF, INPUT_PULLUP);
    pinMode(PIN_SPD_RF, INPUT_PULLUP);
    
    pinMode(PIN_AC_DIF, OUTPUT);
    digitalWrite(PIN_AC_DIF, LOW);
    pinMode(PIN_AC_BUZINA, OUTPUT);
    digitalWrite(PIN_AC_BUZINA, LOW);
    
    // Configura ISR de velocidade
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_LF), isrLF, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_SPD_RF), isrRF, FALLING);

    // Inicializa IMU via Hardware Serial 1 (Com Buffer Expandido)
    Serial1.setRxBufferSize(1024); // Fundamental para não perder dados a 921600 bps
    Serial1.begin(921600, SERIAL_8N1, JY901_RX, JY901_TX);
    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(SensorUartSend);
    WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);

    pinMode(CAN_CS, OUTPUT);
    pinMode(SD_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);
    digitalWrite(SD_CS, HIGH);

    // --- A. BARRAMENTO 1: CAN ---
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    Serial.print("Iniciando CAN... ");
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) { 
        Serial.println("!!! FALHA !!! Tentando novamente...");
        delay(500); 
    }
    CAN0.setMode(MCP_NORMAL);
    Serial.println(">>> SUCESSO!");

    // --- B. BARRAMENTO 2: SD CARD ---
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    Serial.print("Iniciando SD... ");
    if (SD.begin(SD_CS, sdSPI, 4000000)) {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/FECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;pedF_perc;presD;presCM;LF;RF;accX;accY;accZ;dif_atv");
            dataFile.flush();
            Serial.printf(">>> SUCESSO: Gravando em %s\n", nomeArquivo);
        }
    } else {
        Serial.println("!!! FALHA OU AUSENTE !!!");
    }

    xMutexEstado = xSemaphoreCreateMutex();
    filaSD = xQueueCreate(100, sizeof(DadosDinamica));

    // Watchdog Timer
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_deinit();
    esp_task_wdt_init(&twdt_config);

    // Criação das Tasks
    xTaskCreatePinnedToCore(vTaskIMU,        "IMU",   4096, NULL, 4, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskAnalogicos, "Ana",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskVelocidade, "Vel",   3072, NULL, 3, NULL, 0); 
    xTaskCreatePinnedToCore(vTaskSD,         "SD",    4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskRedeCAN,    "CAN",   4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelete(NULL); }

// -------------------------------------------------------------------
// TAREFA 1: IMU Wit-Motion -> Leitura contínua + Mestre do SD
// -------------------------------------------------------------------
void vTaskIMU(void *pvParameters) {
    uint32_t tempoUltimoSD = 0;

    for (;;) {
        // 1. Drena a UART continuamente para evitar overflow do buffer do ESP32
        while (Serial1.available()) {
            WitSerialDataIn(Serial1.read());
        }

        // 2. Verifica se o SDK construiu um pacote válido
        if (s_cDataUpdate) {
            xSemaphoreTake(xMutexEstado, portMAX_DELAY);
            
            estadoAtual.timestamp = millis();
            estadoAtual.acc[0] = sReg[AX] / 32768.0f * 16.0f;
            estadoAtual.acc[1] = sReg[AY] / 32768.0f * 16.0f;
            estadoAtual.acc[2] = sReg[AZ] / 32768.0f * 16.0f;
            
            xSemaphoreGive(xMutexEstado);
            s_cDataUpdate = 0; // Limpa a flag para o SDK voltar a procurar o próximo pacote
        }

        // 3. Envia para a fila do SD no ritmo definido (~70Hz / 14ms)
        if (millis() - tempoUltimoSD >= TAXA_IMU_MS) {
            xSemaphoreTake(xMutexEstado, portMAX_DELAY);
            DadosDinamica p = estadoAtual; 
            xSemaphoreGive(xMutexEstado);
            
            xQueueSend(filaSD, &p, 0);
            tempoUltimoSD = millis();
        }

        // Respiração mínima do RTOS para não travar a CPU (1ms)
        vTaskDelay(1); 
    }
}

// -------------------------------------------------------------------
// TAREFA 2: ANALÓGICOS E PEDAIS
// -------------------------------------------------------------------
void vTaskAnalogicos(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        
        // --- Cálculo Pedal de Freio (Sensor Hall Quadrático) ---
        int mvFreio = analogReadMilliVolts(PIN_PEDAL_F);
        
        // Normaliza para 0.0 a 1.0 (resolvendo a lógica invertida)
        float x_norm = (float)(PEDAL_SOLTO_MV - mvFreio) / (float)(PEDAL_SOLTO_MV - PEDAL_PRESSIONADO_MV);
        
        // Trava entre 0 e 1 para evitar valores fora da escala
        x_norm = constrain(x_norm, 0.0f, 1.0f);
        
        // Aplica a Curva Quadrática e converte para %
        estadoAtual.pedalFreio = (x_norm * x_norm) * 100.0f;

        // --- Restante dos Sensores ---
        estadoAtual.estercamento = (float)analogReadMilliVolts(PIN_STEER); 
        
        int valorCorrente = analogReadMilliVolts(PIN_CUR_DIF);
        estadoAtual.correnteDif = (valorCorrente > 500); 
        
        estadoAtual.presDiant = lerPressaoMPa(PIN_PRES_D);
        estadoAtual.presCM    = lerPressaoMPa(PIN_PRES_CM);
        
        xSemaphoreGive(xMutexEstado);

        vTaskDelay(pdMS_TO_TICKS(TAXA_ANALOG_MS));
    }
}

// -------------------------------------------------------------------
// TAREFA 3: VELOCIDADE DAS RODAS E MONITOR SERIAL
// -------------------------------------------------------------------
void vTaskVelocidade(void *pvParameters) {
    static float buf_LF[TAMANHO_FILTRO] = {0};
    static float buf_RF[TAMANHO_FILTRO] = {0};
    static int indiceFiltro = 0;

    for (;;) {
        unsigned long agora = micros();
        
        noInterrupts();
        unsigned long sDLF = deltaLF; unsigned long sLLF = lastLF;
        unsigned long sDRF = deltaRF; unsigned long sLRF = lastRF;
        interrupts();

        float v_lf_inst = 0, v_rf_inst = 0;

        // Cálculo Roda Esquerda
        unsigned long tempo_desde_LF = agora - sLLF;
        if (tempo_desde_LF < TIMEOUT_US && sDLF > 0) {
            unsigned long t_efetivo = (tempo_desde_LF > sDLF) ? tempo_desde_LF : sDLF;
            v_lf_inst = (CIRC_DIANT * 3600000.0f) / (float(t_efetivo) * DENTES_DIANT);
        }

        // Cálculo Roda Direita
        unsigned long tempo_desde_RF = agora - sLRF;
        if (tempo_desde_RF < TIMEOUT_US && sDRF > 0) {
            unsigned long t_efetivo = (tempo_desde_RF > sDRF) ? tempo_desde_RF : sDRF;
            v_rf_inst = (CIRC_DIANT * 3600000.0f) / (float(t_efetivo) * DENTES_DIANT);
        }

        // Atualiza Buffer da Média Móvel
        buf_LF[indiceFiltro] = v_lf_inst;
        buf_RF[indiceFiltro] = v_rf_inst;
        indiceFiltro = (indiceFiltro + 1) % TAMANHO_FILTRO;

        // Calcula Média
        float soma_LF = 0, soma_RF = 0;
        for (int i = 0; i < TAMANHO_FILTRO; i++) {
            soma_LF += buf_LF[i];
            soma_RF += buf_RF[i];
        }
        
        xSemaphoreTake(xMutexEstado, portMAX_DELAY);
        estadoAtual.v_LF = soma_LF / TAMANHO_FILTRO;
        estadoAtual.v_RF = soma_RF / TAMANHO_FILTRO;
        
        // Print COMPLETO para telemetria no Monitor Serial
        Serial.printf("T:%u | Fr:%3.0f%% | Str:%.0fmV | pD:%.2f | pCM:%.2f | vLF:%.1f | vRF:%.1f | aX:%.2f | aY:%.2f | aZ:%.2f | Dif:%d\n",
                      estadoAtual.timestamp, 
                      estadoAtual.pedalFreio, 
                      estadoAtual.estercamento,
                      estadoAtual.presDiant, 
                      estadoAtual.presCM, 
                      estadoAtual.v_LF,
                      estadoAtual.v_RF,
                      estadoAtual.acc[0],
                      estadoAtual.acc[1],
                      estadoAtual.acc[2],
                      estadoAtual.correnteDif);

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
        // Quem alimenta esta fila agora é a vTaskIMU (~70Hz)
        if (xQueueReceive(filaSD, &s, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%.1f;%.2f;%.2f;%.1f;%.1f;%.2f;%.2f;%.2f;%d\n", 
                    s.timestamp, s.pedalFreio, s.presDiant, 
                    s.presCM, s.v_LF, s.v_RF, s.acc[0], s.acc[1], s.acc[2], s.correnteDif);
                
                // Flush a cada ~1 segundo (71 ciclos de 14ms)
                if (++ct >= 71) {  
                    dataFile.flush(); 
                    ct = 0; 
                }
            }
        }
    }
}

// -------------------------------------------------------------------
// TAREFA 5: REDE CAN
// -------------------------------------------------------------------
void vTaskRedeCAN(void *pvParameters) {
    esp_task_wdt_add(NULL);
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t tempoUltimoEnvio = 0;

    for (;;) {
        esp_task_wdt_reset();
        
        // 1. ESCUTA COMANDOS DA MECU / PAINEL
        while (CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            if (len == 2) {
                int16_t valorInt = (rxBuf[0] << 8) | rxBuf[1];
                
                if (rxId == 0x500) { 
                    digitalWrite(PIN_AC_DIF, (valorInt > 0) ? HIGH : LOW);
                } 
                else if (rxId == 0x501) { 
                    digitalWrite(PIN_AC_BUZINA, (valorInt > 0) ? HIGH : LOW);
                }
            }
        }

        // 2. ENVIA TELEMETRIA (14ms sincronizado com a IMU)
        if (millis() - tempoUltimoEnvio >= TAXA_REDE_CAN_MS) { 
            xSemaphoreTake(xMutexEstado, portMAX_DELAY);
            DadosDinamica p = estadoAtual;
            xSemaphoreGive(xMutexEstado);
                
            enviarMsgCAN(0x400, p.pedalFreio);
            enviarMsgCAN(0x402, p.presDiant);
            enviarMsgCAN(0x403, p.presCM);
            enviarMsgCAN(0x404, p.acc[0]); 
            enviarMsgCAN(0x405, p.acc[1]); 
            enviarMsgCAN(0x406, p.acc[2]); 
            enviarMsgCAN(0x407, (float)p.correnteDif); 
            enviarMsgCAN(0x203, p.v_LF);
            enviarMsgCAN(0x204, p.v_RF);
            
            tempoUltimoEnvio = millis();
        }

        vTaskDelay(1);
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
