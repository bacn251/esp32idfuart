#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "string.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "portmacro.h"
#include "ParseGps.h"
#include "ParseModbus.h"
#include "Wifi.h"
#include "WifiConfig.h"
#include "nvs_flash.h"
// static const int RX_BUF_SIZE = 128;
static const char *APP_TAG = "APP_MAIN";

// Get values from Kconfig
#define APP_VERSION CONFIG_MY_APP_VERSION
#define LED_NUM CONFIG_MY_APP_LED_GPIO
#define LED_ACTIVE_LEVEL CONFIG_MY_APP_LED_ACTIVE_LEVEL
#define GPS_UART_NUM CONFIG_GPS_APP_UART_PORT
#define MODBUS_UART_NUM CONFIG_MODBUS_APP_UART_PORT
#define GPS_UART_BAUD_RATE CONFIG_GPS_APP_UART_BAUD_RATE
#define MODBUS_UART_BAUD_RATE CONFIG_MODBUS_APP_UART_BAUD_RATE
#define GPS_TXD_PIN CONFIG_GPS_APP_UART_TX_PIN
#define GPS_RXD_PIN CONFIG_GPS_APP_UART_RX_PIN
#define MODBUS_TXD_PIN CONFIG_MODBUS_APP_UART_TX_PIN
#define MODBUS_RXD_PIN CONFIG_MODBUS_APP_UART_RX_PIN

#define BUF_SIZE (1024)

typedef struct
{
    int len;
    uint8_t *data;
} uart_packet_t;
typedef enum
{
    UART_ROLE_GPS,
    UART_ROLE_MODBUS
} uart_role_t;

typedef struct
{
    uart_port_t uart_num;
    QueueHandle_t event_queue;
    QueueHandle_t data_queue;
    uart_role_t role;
} uart_ctx_t;
uart_ctx_t uart_gps;
uart_ctx_t uart_modbus;

// void ledConfig(void)
// {
//     gpio_reset_pin(LED_NUM);
//     gpio_set_direction(LED_NUM, GPIO_MODE_OUTPUT);
//     ESP_LOGI(APP_TAG, "LED configured on GPIO %d (Active Level: %s)",
//              LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
// }

void uart_init_config(uart_ctx_t *ctx, uart_port_t uart_num, int baud, int tx_pin, int rx_pin, uart_role_t role)
{
    ctx->uart_num = uart_num;
    ctx->role = role;
    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE, 20, &ctx->event_queue, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ctx->data_queue = xQueueCreate(5, sizeof(uart_packet_t));
}
static void uart_event_task(void *pvParameters)
{
    uart_ctx_t *ctx = (uart_ctx_t *)pvParameters;
    uart_event_t event;
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(ctx->event_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receiving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                uart_packet_t pkt;
                pkt.data = malloc(event.size);
                if (pkt.data == NULL)
                {
                    ESP_LOGE("UART", "Malloc failed");
                    break;
                }
                ESP_LOGI("UART", "Data Event: Bytes=%d", event.size);
                pkt.len = uart_read_bytes(ctx->uart_num, pkt.data, event.size, portMAX_DELAY);
                xQueueSend(ctx->data_queue, &pkt, portMAX_DELAY);
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE("uart", "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ctx->uart_num);
                xQueueReset(ctx->event_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI("uart", "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(ctx->uart_num);
                xQueueReset(ctx->event_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI("uart", "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI("uart", "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI("uart", "uart frame error");
                break;
            // Others
            default:
                ESP_LOGI("uart", "uart event type: %d", event.type);
                break;
            }
        }
    }
}
#define RX_ACC_BUF_SIZE 512
static void uart_process_task(void *pvParameters)
{
    static char gps_buf[GPS_BUF_SIZE];
    static int gps_len = 0;
    uart_ctx_t *ctx = (uart_ctx_t *)pvParameters;
    uart_packet_t pkt;

    static uint8_t acc_buf[512];
    static int acc_len = 0;
    for (;;)
    {
        if (xQueueReceive(ctx->data_queue, &pkt, portMAX_DELAY))
        {
            if (ctx->role == UART_ROLE_GPS)
            {
                for (int i = 0; i < pkt.len; i++)
                {
                    // printf("%02X ", pkt.data[i]);
                    char c = (char)pkt.data[i];
                    if (c == '\n')
                    {
                        gps_buf[gps_len] = '\0';
                        gps_process_line(gps_buf); // process 1 complete line
                        gps_len = 0;
                    }
                    else if (c == '$')
                    {
                        gps_buf[0] = '$';
                        gps_len = 1;
                    }
                    else if (c != '\r')
                    {
                        if (gps_len < GPS_BUF_SIZE - 1)
                            gps_buf[gps_len++] = c;
                        else
                        {
                            ESP_LOGW("GPS", "Line overflow, reset");
                            gps_len = 0;
                        }
                    }
                }
                free(pkt.data);
            }
            else if (ctx->role == UART_ROLE_MODBUS)
            {
                ESP_LOGI("data", "in queue, pkt len=%d, acc_len=%d", pkt.len, acc_len);
                ESP_LOG_BUFFER_HEX("data", pkt.data, pkt.len);
                if (acc_len + pkt.len > RX_ACC_BUF_SIZE)
                {
                    acc_len = 0; // reset if overflow
                }

                memcpy(&acc_buf[acc_len], pkt.data, pkt.len);
                acc_len += pkt.len;

                free(pkt.data);

                // parse loop
                int offset = 0;

                while (1)
                {
                    int frame_len = modbus_get_frame_len(&acc_buf[offset], acc_len - offset);

                    if (frame_len == 0)
                    {
                        // Invalid frame or insufficient data
                        // Skip 1 byte and try to resync
                        if (acc_len - offset > 1)
                        {
                            offset++;
                            continue;
                        }
                        else
                        {
                            break; // Not enough data to continue searching
                        }
                    }

                    if ((acc_len - offset) < frame_len)
                    {
                        break; // Not enough data yet
                    }

                    // check CRC
                    uint16_t crc_calc = modbus_crc16(&acc_buf[offset], frame_len - 2);

                    uint16_t crc_recv = acc_buf[offset + frame_len - 2] |
                                        (acc_buf[offset + frame_len - 1] << 8);
                    ESP_LOGI("MODBUS", "Frame received, len=%d, CRC calc=0x%04X", frame_len, crc_calc);
                    ESP_LOGI("MODBUS", "Received CRC: 0x%04X", crc_recv);
                    if (crc_calc == crc_recv)
                    {
                        ESP_LOGI("MODBUS", "Frame OK, len=%d", frame_len);
                        uart_write_bytes(ctx->uart_num, (char *)&acc_buf[offset], frame_len);
                    }
                    else
                    {
                        ESP_LOGE("MODBUS", "CRC ERROR");
                    }

                    offset += frame_len;
                }
                if (offset > 0)
                {
                    memmove(acc_buf, &acc_buf[offset], acc_len - offset);
                    acc_len -= offset;
                }
            }
        }
    }
}
void app_main(void)
{
    ESP_LOGI(APP_TAG, "========== Application Started ==========");
    ESP_LOGI(APP_TAG, "App Version: %s", APP_VERSION);
    ESP_LOGI(APP_TAG, "LED GPIO: %d (Active: %s)", LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
    ESP_LOGI(APP_TAG, "UART Port: %d, Baud: %d, TX: %d, RX: %d",
             GPS_UART_NUM, GPS_UART_BAUD_RATE, GPS_TXD_PIN, GPS_RXD_PIN);
    ESP_LOGI(APP_TAG, "UART Port: %d, Baud: %d, TX: %d, RX: %d",
             MODBUS_UART_NUM, MODBUS_UART_BAUD_RATE, MODBUS_TXD_PIN, MODBUS_RXD_PIN);
    ESP_LOGI(APP_TAG, "==========================================");

    /* --- NVS init (required by WiFi driver) --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* --- WiFi start:
     *   - Has credentials in NVS   → connect STA
     *   - No credentials yet        → SoftAP config mode (first time)
     *   - Hold BOOT >= 10s AT ANY TIME → clear NVS → restart → SoftAP
     * --------------------------------------------------------------- */
    ESP_LOGI(APP_TAG, "Starting WiFi...");
    wifi_start();

    if (isConnected) {
        ESP_LOGI(APP_TAG, "[WiFi] Mode: STA  →  connected ✓");
    } else if (isAPMode) {
        ESP_LOGW(APP_TAG, "[WiFi] Mode: SoftAP  →  SSID: \"%s\"  IP: 192.168.4.1", SOFTAP_SSID);
        ESP_LOGW(APP_TAG, "[WiFi] Access http://192.168.4.1 to configure WiFi.");
    } else {
        ESP_LOGE(APP_TAG, "[WiFi] WiFi connection failed – hold BOOT >= 10s to reconfigure.");
    }

    /* --- Start background task monitoring BOOT button ---
     *  Hold button >= 10s → clear NVS WiFi → restart → enter SoftAP
     * ---------------------------------------------------- */
    wifi_button_monitor_start();
    uart_init_config(&uart_gps, GPS_UART_NUM, GPS_UART_BAUD_RATE, GPS_TXD_PIN, GPS_RXD_PIN, UART_ROLE_GPS);
    uart_init_config(&uart_modbus, MODBUS_UART_NUM, MODBUS_UART_BAUD_RATE, MODBUS_TXD_PIN, MODBUS_RXD_PIN, UART_ROLE_MODBUS);
    xTaskCreate(uart_event_task, "uart_gps_event_task", 3072, &uart_gps, 12, NULL);
    xTaskCreate(uart_process_task, "uart_gps_process_task", 8192, &uart_gps, 11, NULL);

    xTaskCreate(uart_event_task, "uart_modbus_event_task", 3072, &uart_modbus, 12, NULL);
    xTaskCreate(uart_process_task, "uart_modbus_process_task", 8192, &uart_modbus, 11, NULL);
}
