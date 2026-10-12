#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

// --- CONFIGURATION ---
#define CALLSIGN            "K1ABC"   // Set your callsign
#define GRID_SQUARE         "FM19"    // Set your 4-char Grid Locator
#define POWER_DBM           23        // Transmission Power in dBm (~200mW)
#define XTAL_FREQ_HZ        25000000ULL 

// Hardware Pins
#define I2C_MASTER_SDA_IO   5
#define I2C_MASTER_SCL_IO   6
#define I2C_MASTER_NUM      I2C_NUM_0

#define GPS_UART_NUM        UART_NUM_1
#define GPS_TX_PIN          21  
#define GPS_RX_PIN          20  
#define GPS_PPS_PIN         0   
#define GPS_BAUD_RATE       9600

#define PCF8574_I2C_ADDR    0x20
#define SI5351_I2C_ADDR     0x60

static const char *TAG = "WSPR_GPS_BEACON";
static i2c_master_dev_handle_t pcf_handle;
static i2c_master_dev_handle_t si_handle;
static uint8_t wspr_symbols[162];

// Active Operating Parameters
static uint64_t current_base_freq_hz = 14097100ULL; // Default 20m Dial + 1500Hz
static uint8_t current_pcf_mask = 0x06;            // Default 30m-20m Matrix byte

static const uint8_t wspr_sync_vector[162] = {
    1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,
    0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,
    0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
    0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,
    0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,
    0,0
};

// --- I2C BUS RECOVERY ---
static void i2c_bus_recovery(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pin_bit_mask = (1ULL << I2C_MASTER_SDA_IO) | (1ULL << I2C_MASTER_SCL_IO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        esp_rom_delay_us(10);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(10);
    }
}

// --- SI5351 DRIVER ---
static esp_err_t si5351_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(si_handle, buf, 2, 100 / portTICK_PERIOD_MS);
}

void si5351_set_frequency_direct(uint64_t freq_hz_milli) {
    uint32_t ms_divider = 50;
    uint64_t vco_freq_milli = freq_hz_milli * ms_divider;

    uint64_t a = vco_freq_milli / (XTAL_FREQ_HZ * 1000ULL);
    uint64_t rem = vco_freq_milli % (XTAL_FREQ_HZ * 1000ULL);

    uint32_t c = 1048575;
    uint32_t b = (uint32_t)((rem * c) / (XTAL_FREQ_HZ * 1000ULL));

    uint32_t p1 = 128 * a + (128 * b) / c - 512;
    uint32_t p2 = 128 * b - c * ((128 * b) / c);
    uint32_t p3 = c;

    si5351_write_reg(26, (p3 >> 8) & 0xFF);
    si5351_write_reg(27, p3 & 0xFF);
    si5351_write_reg(28, (p1 >> 16) & 0x03);
    si5351_write_reg(29, (p1 >> 8) & 0xFF);
    si5351_write_reg(30, p1 & 0xFF);
    si5351_write_reg(31, ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F));
    si5351_write_reg(32, (p2 >> 8) & 0xFF);
    si5351_write_reg(33, p2 & 0xFF);
}

void si5351_power_down(bool shutdown) {
    if (shutdown) {
        si5351_write_reg(3, 0xFF);  
        si5351_write_reg(16, 0x80); 
    } else {
        si5351_write_reg(16, 0x4F); 
        si5351_write_reg(3, 0xFE);  
    }
}

void si5351_init(void) {
    si5351_power_down(true);
    si5351_write_reg(183, 0x80); 
    si5351_write_reg(42, 0x00);
    si5351_write_reg(43, 0x01);
    si5351_write_reg(44, 0x00);
    si5351_write_reg(45, 0x17);
    si5351_write_reg(46, 0x00);
    si5351_write_reg(47, 0x00);
    si5351_write_reg(48, 0x00);
    si5351_write_reg(49, 0x00);
}

// --- STANDARD WSPR ENCODER ---
static int wspr_call_char_code(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'Z') return c - 'A' + 10;
    if (c == ' ') return 36; 
    return -1;
}

static uint8_t wspr_reverse8(uint8_t x) {
    x = (uint8_t)(((x & 0xF0U) >> 4) | ((x & 0x0FU) << 4));
    x = (uint8_t)(((x & 0xCCU) >> 2) | ((x & 0x33U) << 2));
    x = (uint8_t)(((x & 0xAAU) >> 1) | ((x & 0x55U) << 1));
    return x;
}

static int wspr_pack_message(const char *callsign, const char *grid, uint8_t power_dbm, uint64_t *message_out) {
    if (!callsign || !grid || !message_out) return -1;

    size_t len = strlen(callsign);
    if (len < 3 || len > 6 || strlen(grid) != 4) return -1;

    char call[7] = {0};
    char input[7] = {0};

    for (size_t i = 0; i < len; i++) {
        char c = callsign[i];
        input[i] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
    }

    if (input[2] < '0' || input[2] > '9') {
        if (len >= 6) return -1;
        call[0] = ' ';
        memcpy(call + 1, input, len);
    } else {
        memcpy(call, input, len);
    }

    int c0 = wspr_call_char_code(call[0]);
    int c1 = wspr_call_char_code(call[1]);
    int c2 = wspr_call_char_code(call[2]);

    if (c0 < 0 || c1 < 0 || c1 >= 36 || c2 < 0 || c2 > 9) return -1;

    int suffix[3];
    for (int i = 0; i < 3; i++) {
        int v = wspr_call_char_code(call[i + 3]);
        if (v < 10 || v > 36) return -1;
        suffix[i] = v - 10;
    }

    uint64_t ncall = (uint64_t)c0;
    ncall = ncall * 36 + (uint64_t)c1;
    ncall = ncall * 10 + (uint64_t)c2;
    ncall = ncall * 27 + (uint64_t)suffix[0];
    ncall = ncall * 27 + (uint64_t)suffix[1];
    ncall = ncall * 27 + (uint64_t)suffix[2];

    if (ncall >= (1ULL << 28)) return -1;

    char g[5];
    for (int i = 0; i < 4; i++) {
        char c = grid[i];
        g[i] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
    }
    g[4] = '\0';

    if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R' ||
        g[2] < '0' || g[2] > '9' || g[3] < '0' || g[3] > '9') return -1;

    int a = g[0] - 'A';
    int b = g[1] - 'A';
    int c = g[2] - '0';
    int d = g[3] - '0';

    uint32_t ngrid = (uint32_t)((179 - 10 * a - c) * 180 + 10 * b + d);

    static const int8_t power_correction[10] = {0, -1, 1, 0, -1, 2, 1, 0, -1, 1};
    if (power_dbm > 60) return -1;

    int encoded_power = (int)power_dbm + power_correction[power_dbm % 10] + 64;
    if (encoded_power < 0 || encoded_power > 127) return -1;

    *message_out = (ncall << 22) | ((uint64_t)ngrid << 7) | (uint64_t)encoded_power;
    return 0;
}

int wspr_encode(const char *callsign, const char *grid, uint8_t power_dbm, uint8_t *symbols_out) {
    uint64_t message;
    uint8_t convolutional[162];
    uint8_t interleaved[162];

    if (symbols_out == NULL) return -1;
    if (wspr_pack_message(callsign, grid, power_dbm, &message) != 0) return -1;

    uint32_t reg = 0;
    int out_idx = 0;

    for (int i = 0; i < 81; i++) {
        uint8_t bit = (i < 50) ? (uint8_t)((message >> (49 - i)) & 1ULL) : 0;
        reg = (reg << 1) | bit;

        convolutional[out_idx++] = (uint8_t)(__builtin_popcount(reg & 0xF2D05351U) & 1);
        convolutional[out_idx++] = (uint8_t)(__builtin_popcount(reg & 0xE4613C47U) & 1);
    }

    int src_idx = 0;
    for (int i = 0; i < 256; i++) {
        uint8_t dest = wspr_reverse8((uint8_t)i);
        if (dest < 162) {
            interleaved[dest] = convolutional[src_idx++];
        }
    }

    if (src_idx != 162) return -1;

    for (int i = 0; i < 162; i++) {
        symbols_out[i] = (uint8_t)(2 * interleaved[i] + wspr_sync_vector[i]);
    }

    return 0;
}

// --- GPS UART & NMEA PARSER ---
static void gps_hardware_init(void) {
    gpio_config_t pps_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPS_PPS_PIN),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&pps_conf);

    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, 1024, 0, 0, NULL, 0));
}

static time_t utc_to_epoch(const struct tm *tm) {
    int year = tm->tm_year + 1900;
    int mon = tm->tm_mon + 1; 
    int mday = tm->tm_mday;
    int hour = tm->tm_hour;
    int min = tm->tm_min;
    int sec = tm->tm_sec;

    if (mon <= 2) {
        mon += 12;
        year -= 1;
    }

    int32_t days = (365 * year) + (year / 4) - (year / 100) + (year / 400) 
                 + (306 * (mon + 1) / 10) + mday - 719591;

    return (time_t)(days * 86400 + hour * 3600 + min * 60 + sec);
}

static bool parse_gprmc(const char* line) {
    if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0) {
        return false;
    }

    char buf[128];
    strncpy(buf, line, sizeof(buf));
    
    char *tokens[13];
    int token_idx = 0;
    char *token = strtok(buf, ",");
    
    while (token != NULL && token_idx < 13) {
        tokens[token_idx++] = token;
        token = strtok(NULL, ",");
    }

    if (token_idx > 9 && strcmp(tokens[2], "A") == 0) {
        int time_raw = atoi(tokens[1]);
        int date_raw = atoi(tokens[9]);

        struct tm tm_time = {0};
        tm_time.tm_hour = time_raw / 10000;
        tm_time.tm_min  = (time_raw / 100) % 100;
        tm_time.tm_sec  = time_raw % 100;

        tm_time.tm_mday = date_raw / 10000;
        tm_time.tm_mon  = ((date_raw / 100) % 100) - 1;
        tm_time.tm_year = (date_raw % 100) + 100;

        time_t t = utc_to_epoch(&tm_time);
        struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        return true;
    }
    return false;
}

static void gps_task(void *pvParameters) {
    uint8_t data[256];
    char line_buf[128];
    int line_pos = 0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line_buf[line_pos] = '\0';
                        parse_gprmc(line_buf);
                        line_pos = 0;
                    }
                } else if (line_pos < sizeof(line_buf) - 1) {
                    line_buf[line_pos++] = c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- NON-BLOCKING INTERACTIVE BAND SELECTOR ---
static void select_band_menu(void) {
    // Install UART driver on Console UART (UART 0) to allow instant non-blocking input
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    printf("\n=======================================================\n");
    printf("         BAND SELECTION MENU         \n");
    printf("=======================================================\n");
    printf(" [1] 30m Band (10.1387 MHz Dial - 15.5MHz Cutoff )\n");
    printf(" [2] 20m Band (14.0956 MHz Dial - 15.5MHz Cutoff )\n");
    printf(" [3] 17m Band (18.1046 MHz Dial - 29.0MHz Cutoff )\n");
    printf(" [4] 15m Band (21.0946 MHz Dial - 29.0MHz Cutoff )\n");
    printf(" [5] 12m Band (24.9246 MHz Dial - 32.0MHz Cutoff )\n");
    printf(" [6] 10m Band (28.1246 MHz Dial - 32.0MHz Cutoff )\n");
    printf(" [7] BYPASS Mode (Unfiltered Direct Trace        )\n");
    printf("-------------------------------------------------------\n");
    printf("Select Wavelength/Mode [1-7]: ");
    fflush(stdout);

    uint8_t ch = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, 100 / portTICK_PERIOD_MS);
        if (len > 0 && ch >= '1' && ch <= '7') {
            printf("%c\n", ch);
            break;
        }
    }

    switch (ch) {
        case '1':
            current_base_freq_hz = 10140200ULL; // 30m Dial (10.1387 MHz) + 1500Hz
            current_pcf_mask = 0x06;            // PCF8574 outputs 0x06 (P2=1, P1=1)
            ESP_LOGI(TAG, "Selected: 30m Band (10.1387 MHz) -> PCF8574 Control Byte: 0x06");
            break;
        case '2':
            current_base_freq_hz = 14097100ULL; // 20m Dial (14.0956 MHz) + 1500Hz
            current_pcf_mask = 0x06;            // PCF8574 outputs 0x06 (P2=1, P1=1)
            ESP_LOGI(TAG, "Selected: 20m Band (14.0956 MHz) -> PCF8574 Control Byte: 0x06");
            break;
        case '3':
            current_base_freq_hz = 18106100ULL; // 17m Dial (18.1046 MHz) + 1500Hz
            current_pcf_mask = 0x02;            // PCF8574 outputs 0x02 (P2=0, P1=1)
            ESP_LOGI(TAG, "Selected: 17m Band (18.1046 MHz) -> PCF8574 Control Byte: 0x02");
            break;
        case '4':
            current_base_freq_hz = 21096100ULL; // 15m Dial (21.0946 MHz) + 1500Hz
            current_pcf_mask = 0x02;            // PCF8574 outputs 0x02 (P2=0, P1=1)
            ESP_LOGI(TAG, "Selected: 15m Band (21.0946 MHz) -> PCF8574 Control Byte: 0x02");
            break;
        case '5':
            current_base_freq_hz = 24926100ULL; // 12m Dial (24.9246 MHz) + 1500Hz
            current_pcf_mask = 0x04;            // PCF8574 outputs 0x04 (P2=1, P1=0)
            ESP_LOGI(TAG, "Selected: 12m Band (24.9246 MHz) -> PCF8574 Control Byte: 0x04");
            break;
        case '6':
            current_base_freq_hz = 28126100ULL; // 10m Dial (28.1246 MHz) + 1500Hz
            current_pcf_mask = 0x04;            // PCF8574 outputs 0x04 (P2=1, P1=0)
            ESP_LOGI(TAG, "Selected: 10m Band (28.1246 MHz) -> PCF8574 Control Byte: 0x04");
            break;
        case '7':
            current_base_freq_hz = 14097100ULL; // Default 20m frequency
            current_pcf_mask = 0x00;             // PCF8574 outputs 0x00 (P2=0, P1=0) for RF switch Bypass
            ESP_LOGW(TAG, "=======================================================");
            ESP_LOGW(TAG, " WARNING: BYPASS MODE SELECTED! TRANSMISSION IS UNFILTERED!");
            ESP_LOGW(TAG, " Ensure an external Low-Pass Filter is connected to avoid");
            ESP_LOGW(TAG, " transmitting square-wave harmonics across the spectrum.");
            ESP_LOGW(TAG, "=======================================================");
            break;
        default:
            ESP_LOGW(TAG, "Invalid selection. Defaulting to 20m Band (0x06).");
            current_base_freq_hz = 14097100ULL;
            current_pcf_mask = 0x06;
            break;
    }

    // Send control byte directly to PCF8574 GPIO extender
    i2c_master_transmit(pcf_handle, &current_pcf_mask, 1, 1000 / portTICK_PERIOD_MS);
}

// --- MAIN APP ---
void app_main(void) {
    i2c_bus_recovery();

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t pcf_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = PCF8574_I2C_ADDR, .scl_speed_hz = 100000};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &pcf_cfg, &pcf_handle));

    i2c_device_config_t si_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SI5351_I2C_ADDR, .scl_speed_hz = 100000};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &si_cfg, &si_handle));

    vTaskDelay(pdMS_TO_TICKS(100));

    // Interactive band menu selection
    select_band_menu();

    si5351_init();

    if (wspr_encode(CALLSIGN, GRID_SQUARE, POWER_DBM, wspr_symbols) != 0) {
        ESP_LOGE(TAG, "WSPR encoding failed: check callsign, grid, and power");
        return;
    }
    ESP_LOGI(TAG, "WSPR encoding successful: %s %s %d dBm", CALLSIGN, GRID_SQUARE, POWER_DBM);

    gps_hardware_init();
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Waiting for GPS UTC lock...");

    time_t now = 0;
    struct tm timeinfo = {0};

    while (1) {
        time(&now);
        gmtime_r(&now, &timeinfo);

        if (timeinfo.tm_year > (2020 - 1900)) {
            ESP_LOGI(TAG, "GPS UTC Locked! Time: %02d:%02d:%02d UTC", 
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    const uint32_t tone_offsets_milli[4] = {0, 1465, 2930, 4395};

    while (1) {
        time(&now);
        gmtime_r(&now, &timeinfo);

        if (timeinfo.tm_min % 2 == 0 && timeinfo.tm_sec == 0) {
            ESP_LOGI(TAG, "GPS Trigger! Transmitting at %02d:%02d:%02d UTC...", 
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

            si5351_power_down(false);

            int64_t tx_start_us = esp_timer_get_time();

            for (int i = 0; i < 162; i++) {
                uint8_t symbol = wspr_symbols[i];
                uint64_t target_freq_milli = (current_base_freq_hz * 1000ULL) + tone_offsets_milli[symbol];
                
                si5351_set_frequency_direct(target_freq_milli);

                int64_t target_symbol_time_us = tx_start_us + (int64_t)(i + 1) * 682667LL;
                while (esp_timer_get_time() < target_symbol_time_us) {
                    vTaskDelay(1);
                }
            }

            si5351_power_down(true);
            ESP_LOGI(TAG, "Transmission Complete. Waiting for next even UTC minute.");
            
            vTaskDelay(pdMS_TO_TICKS(50000));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}