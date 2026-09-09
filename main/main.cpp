#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "driver/uart.h"

// Inclusões do LoRaWAN (LMIC)
#include "lmic.h"
#include "hal/hal.h"

static const char *TAG = "TRACKER_SIIEPE";

// ==============================================================================
// CONFIGURAÇÕES DE PINOS (GPS e LORA SX1276)
// ==============================================================================
// GPS
#define ESP_RX_PIN 4
#define ESP_TX_PIN 5
#define GPS_UART_PORT UART_NUM_1
#define BUF_SIZE 1024

// LORA SX1276
#define LORA_NSS 18
#define LORA_RST 10
#define LORA_DIO0 11
#define LORA_DIO1 3
// SCK = 6, MOSI = 7, MISO = 2 (Esses serão usados pelo barramento SPI da placa)

const lmic_pinmap lmic_pins = {
    .nss = LORA_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_DIO0, LORA_DIO1, LMIC_UNUSED_PIN},
};

// ==============================================================================
// CHAVES DA TTN (OTAA)
// ==============================================================================
// AppEUI (LSB)
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// DevEUI (LSB)
static const u1_t PROGMEM DEVEUI[8] = { 0x87, 0x8F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// AppKey (MSB)
static const u1_t PROGMEM APPKEY[16] = { 0xC5, 0x7B, 0x7B, 0xEC, 0x2E, 0xC0, 0xD6, 0x60, 0x95, 0xE9, 0x3E, 0x20, 0xC9, 0x8D, 0x15, 0x56 };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ==============================================================================
// VARIÁVEIS GLOBAIS
// ==============================================================================
#define LOG_FILE_PATH "/spiffs/trajeto_pelotas.csv"
#define TX_INTERVAL 30 // Segundos entre os envios do LoRa

float current_lat = 0.0;
float current_lon = 0.0;
float current_hdop = 0.0;
bool has_fix = false;

static osjob_t sendjob;
uint8_t ttn_payload[9];

// ==============================================================================
// FUNÇÕES DO GPS E LITTLEFS
// ==============================================================================
void init_gps_uart(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, ESP_TX_PIN, ESP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void init_littlefs(void) {
    esp_vfs_littlefs_conf_t conf = {}; 
    conf.base_path = "/spiffs";
    conf.partition_label = "storage";
    conf.format_if_mount_failed = true;
    esp_vfs_littlefs_register(&conf);
}

void append_to_csv(const char *data) {
    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (f != NULL) {
        fprintf(f, "%s\n", data);
        fclose(f);
    }
}

void dump_csv_file(void) {
    ESP_LOGI(TAG, "=== INÍCIO DA EXTRAÇÃO DO CSV ===");
    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) return;
    char line[128];
    while (fgets(line, sizeof(line), f) != NULL) {
        printf("%s", line);
    }
    fclose(f);
    ESP_LOGI(TAG, "=== FIM DA EXTRAÇÃO ===");
}

void parse_nmea_sentence(char *line) {
    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        char *fields[15];
        int count = 0;
        fields[count++] = line;
        for (char *p = line; *p != '\0'; p++) {
            if (*p == ',') {
                *p = '\0';
                fields[count++] = p + 1;
                if (count >= 15) break;
            }
        }
        
        if (count > 8 && atoi(fields[6]) > 0) {
            has_fix = true;
            
            float lat_raw = atof(fields[2]);
            int lat_deg = lat_raw / 100;
            current_lat = lat_deg + ((lat_raw - (lat_deg * 100)) / 60.0);
            if (fields[3][0] == 'S') current_lat = -current_lat;

            float lon_raw = atof(fields[4]);
            int lon_deg = lon_raw / 100;
            current_lon = lon_deg + ((lon_raw - (lon_deg * 100)) / 60.0);
            if (fields[5][0] == 'W') current_lon = -current_lon;

            current_hdop = atof(fields[8]);
        } else {
            has_fix = false;
        }
    }
}

// ==============================================================================
// FUNÇÕES DO LORAWAN E TTN
// ==============================================================================
void pack_lora_payload() {
    int32_t lat_int = current_lat * 1000000;
    int32_t lon_int = current_lon * 1000000;
    uint8_t hdop_int = (uint8_t)(current_hdop * 10); // Multiplica por 10 para salvar a casa decimal

    ttn_payload[0] = (lat_int >> 24) & 0xFF;
    ttn_payload[1] = (lat_int >> 16) & 0xFF;
    ttn_payload[2] = (lat_int >> 8) & 0xFF;
    ttn_payload[3] = lat_int & 0xFF;

    ttn_payload[4] = (lon_int >> 24) & 0xFF;
    ttn_payload[5] = (lon_int >> 16) & 0xFF;
    ttn_payload[6] = (lon_int >> 8) & 0xFF;
    ttn_payload[7] = lon_int & 0xFF;

    ttn_payload[8] = hdop_int;
}

void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        ESP_LOGW(TAG, "LoRa: TX pendente, aguardando...");
    } else {
        if (has_fix) {
            pack_lora_payload();
            LMIC_setTxData2(1, ttn_payload, sizeof(ttn_payload), 0);
            ESP_LOGI(TAG, "LoRa: Pacote enviado para fila TX! (Lat: %.6f, Lon: %.6f)", current_lat, current_lon);
        } else {
            ESP_LOGW(TAG, "LoRa: Sem GPS Fix. Pulando transmissão.");
            // Reagenda mesmo sem fix para manter o ciclo vivo
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        }
    }
}

void onEvent(ev_t ev) {
    switch(ev) {
        case EV_JOINING:
            ESP_LOGI(TAG, "LoRa: Tentando OTAA Join na TTN...");
            break;
        case EV_JOINED:
            ESP_LOGI(TAG, "LoRa: Conectado à TTN com SUCESSO! 🎉");
            LMIC_setLinkCheckMode(0); // Desativa LinkCheck para não poluir
            break;
        case EV_TXCOMPLETE:
            ESP_LOGI(TAG, "LoRa: Transmissão Concluída! Aguardando próxima janela...");
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_JOIN_FAILED:
            ESP_LOGE(TAG, "LoRa: Falha no Join OTAA. Verifique chaves ou cobertura.");
            break;
        default:
            break;
    }
}

// ==============================================================================
// TASKS DO FREERTOS
// ==============================================================================
void gps_logger_task(void *pvParameters) {
    uint8_t rx_buffer[64];
    char line_buffer[128];
    int line_pos = 0;
    int f_cnt = 1;

    FILE *f = fopen(LOG_FILE_PATH, "r");
    if (f == NULL) append_to_csv("f_cnt,latitude,longitude,hdop");
    else fclose(f);

    TickType_t last_save_time = xTaskGetTickCount();

    while (1) {
        int len = uart_read_bytes(GPS_UART_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++) {
            if (rx_buffer[i] == '\n') {
                line_buffer[line_pos] = '\0';
                parse_nmea_sentence(line_buffer);
                line_pos = 0;
            } else if (rx_buffer[i] != '\r' && line_pos < sizeof(line_buffer) - 1) {
                line_buffer[line_pos++] = rx_buffer[i];
            }
        }

        // Salva no CSV a cada 30 segundos, independentemente do envio LoRa
        if ((xTaskGetTickCount() - last_save_time) > pdMS_TO_TICKS(30000)) {
            if (has_fix) {
                char csv_line[128];
                snprintf(csv_line, sizeof(csv_line), "%d,%.6f,%.6f,%.2f", f_cnt++, current_lat, current_lon, current_hdop);
                append_to_csv(csv_line);
                ESP_LOGI(TAG, "CSV: ✅ Salvo -> %s", csv_line);
            }
            last_save_time = xTaskGetTickCount();
        }
    }
}

void lmic_task(void *pvParameters) {
    os_init();
    LMIC_reset();
    
    // Potência máxima para maximizar o mapeamento (DR_SF10 - Spreading Factor 10)
    LMIC_setDrTxpow(DR_SF10, 14); 

    do_send(&sendjob);

    while (1) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(10)); // Evita que o watchdog reinicie o chip
    }
}

// ==============================================================================
// MAIN APP
// ==============================================================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Firmware LoRa Tracker (UFPel/SIIEPE) - OTAA");

    init_littlefs();
    init_gps_uart();
    
    dump_csv_file(); 

    // Cria as tarefas: O LMIC precisa de bastante memória de pilha (stack)
    xTaskCreate(gps_logger_task, "gps_logger", 4096, NULL, 5, NULL);
    xTaskCreate(lmic_task, "lmic_task", 8192, NULL, 5, NULL);
}