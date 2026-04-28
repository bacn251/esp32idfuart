#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "string.h"
#include "driver/gpio.h"
#include "unity_internals.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "portmacro.h"
#include "ParseGps.h"
// static const int RX_BUF_SIZE = 128;
static const char *APP_TAG = "APP_MAIN";

// Get values from Kconfig
#define APP_VERSION CONFIG_MY_APP_VERSION
#define LED_NUM CONFIG_MY_APP_LED_GPIO
#define LED_ACTIVE_LEVEL CONFIG_MY_APP_LED_ACTIVE_LEVEL
#define UART_NUM CONFIG_MY_APP_UART_PORT
#define UART_BAUD_RATE CONFIG_MY_APP_UART_BAUD_RATE
#define TXD_PIN CONFIG_MY_APP_UART_TX_PIN
#define RXD_PIN CONFIG_MY_APP_UART_RX_PIN

#define BUF_SIZE (1024)

static QueueHandle_t event_queue;

typedef struct
{
    int len;
    uint8_t *data;
} uart_packet_t;

static QueueHandle_t data_queue;

// void ledConfig(void)
// {
//     gpio_reset_pin(LED_NUM);
//     gpio_set_direction(LED_NUM, GPIO_MODE_OUTPUT);
//     ESP_LOGI(APP_TAG, "LED configured on GPIO %d (Active Level: %s)",
//              LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
// }

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE, 20, &event_queue, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(event_queue, (void *)&event, (TickType_t)portMAX_DELAY))
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
                pkt.len = uart_read_bytes(UART_NUM, pkt.data, event.size, portMAX_DELAY);
                xQueueSend(data_queue, &pkt, portMAX_DELAY);
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE("uart", "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
                xQueueReset(event_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI("uart", "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM);
                xQueueReset(event_queue);
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
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
static int modbus_get_frame_len(uint8_t *buf, int len)
{
    if (len < 2)
        return 0;

    uint8_t func = buf[1];

    switch (func)
    {
    case 0x03: // Read Holding Registers REQUEST → fixed 8 bytes
        if (len < 3)
            return 0;
        return 3 + buf[2] + 2;
    case 0x06: // Write Single Register → fixed 8 bytes
        return 8;

    case 0x83: // Read Holding Registers RESPONSE (exception)
        return 5;

        // Nếu bạn cần parse 0x03 RESPONSE (có byte count):
        // case 0x03 response sẽ có dạng: 01 03 04 xx xx xx xx CRC CRC
        // Phân biệt request/response bằng context (master/slave side)

    default:
        return 0;
    }
}
#define RX_ACC_BUF_SIZE 512
static void uart_process_task(void *pvParameters)
{
    static char gps_buf[GPS_BUF_SIZE];
    static int gps_len = 0;

    uart_packet_t pkt;

    for (;;)
    {
        if (xQueueReceive(data_queue, &pkt, portMAX_DELAY))
        {
            for (int i = 0; i < pkt.len; i++)
            {
                // printf("%02X ", pkt.data[i]);
                char c = (char)pkt.data[i];
                if (c == '\n')
                {
                    gps_buf[gps_len] = '\0';
                    gps_process_line(gps_buf); // xử lý 1 line hoàn chỉnh
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
            // 
            // append data
            // ESP_LOGI("data", "in queue, pkt len=%d, acc_len=%d", pkt.len, acc_len);
            // ESP_LOG_BUFFER_HEX("data", pkt.data, pkt.len);
            // if (pkt.data[0] == '$')
            // {
            //     ESP_LOGI("data", "NMEA sentence received, try parse");
            //     gps_process_line((char *)pkt.data);
            //     free(pkt.data);
            // }
            // else
            // {
            //     if (acc_len + pkt.len > RX_ACC_BUF_SIZE)
            //     {
            //         acc_len = 0; // reset nếu overflow
            //     }

            //     memcpy(&acc_buf[acc_len], pkt.data, pkt.len);
            //     acc_len += pkt.len;

            //     free(pkt.data);

            //     // parse loop
            //     int offset = 0;

            //     while (1)
            //     {
            //         int frame_len = modbus_get_frame_len(&acc_buf[offset], acc_len - offset);

            //         if (frame_len == 0 || (acc_len - offset) < frame_len)
            //         {
            //             break; // chưa đủ data
            //         }

            //         // check CRC
            //         uint16_t crc_calc = modbus_crc16(&acc_buf[offset], frame_len - 2);

            //         uint16_t crc_recv = acc_buf[offset + frame_len - 2] |
            //                             (acc_buf[offset + frame_len - 1] << 8);
            //         ESP_LOGI("MODBUS", "Frame received, len=%d, CRC calc=0x%04X", frame_len, crc_calc);
            //         ESP_LOGI("MODBUS", "Received CRC: 0x%04X", crc_recv);
            //         if (crc_calc == crc_recv)
            //         {
            //             ESP_LOGI("MODBUS", "Frame OK, len=%d", frame_len);

            //             // 👉 xử lý frame ở đây
            //             // ví dụ echo lại
            //             uart_write_bytes(UART_NUM, (char *)&acc_buf[offset], frame_len);
            //         }
            //         else
            //         {
            //             ESP_LOGE("MODBUS", "CRC ERROR");
            //         }

            //         offset += frame_len;
            //     }

            //     // shift buffer còn lại
            //     if (offset > 0)
            //     {
            //         memmove(acc_buf, &acc_buf[offset], acc_len - offset);
            //         acc_len -= offset;
            //     }
            // }
            // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            // printf("Stack remain: %u bytes\n", uxHighWaterMark * sizeof(StackType_t));
        }
    }
}
// static void uart_process_task(void *pvParameters)
// {
//     uart_packet_t pkt;
//     for (;;)
//     {
//         if (xQueueReceive(data_queue, &pkt, portMAX_DELAY))
//         {
//             ESP_LOGI("uart", "processing data");
//             uart_write_bytes(UART_NUM, pkt.data, pkt.len);
//             free(pkt.data);
//         }
//     }
// }
// static void tx_task(void *arg)
// {
//     uint8_t *data = (uint8_t *)malloc(150);
//     while (1)
//     {
//         int len = sprintf((char *)data, "Data Received is '%s' \n", RxData);
//         uart_write_bytes(UART_NUM, data, len);
//         vTaskSuspend(NULL);
//     }
//     free(data);
// }

// static void rx_task(void *arg)
// {
//     int ledState = 0;
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     while (1)
//     {
//         const int rxBytes = uart_read_bytes(UART_NUM, RxData, RX_BUF_SIZE, 200 / portTICK_PERIOD_MS);
//         if (rxBytes > 0)
//         {
//             RxData[rxBytes] = 0;
//             if (strcmp((char *)RxData, "ON") == 0)
//                 ledState = LED_ACTIVE_LEVEL ? 1 : 0;
//             else if (strcmp((char *)RxData, "OFF") == 0)
//                 ledState = LED_ACTIVE_LEVEL ? 0 : 1;
//             gpio_set_level(LED_NUM, ledState);
//             vTaskResume(txTaskHandle);
//             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, RxData);
//             // ESP_LOG_BUFFER_HEX(RX_TASK_TAG, RxData, rxBytes);
//         }
//     }
// }

void app_main(void)
{
    ESP_LOGI(APP_TAG, "========== Application Started ==========");
    ESP_LOGI(APP_TAG, "App Version: %s", APP_VERSION);
    ESP_LOGI(APP_TAG, "LED GPIO: %d (Active: %s)", LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
    ESP_LOGI(APP_TAG, "UART Port: %d, Baud: %d, TX: %d, RX: %d",
             UART_NUM, UART_BAUD_RATE, TXD_PIN, RXD_PIN);
    ESP_LOGI(APP_TAG, "==========================================");

    init();
    // ledConfig();
    data_queue = xQueueCreate(5, sizeof(uart_packet_t));

    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 12, NULL);
    xTaskCreate(uart_process_task, "uart_process_task", 8192, NULL, 11, NULL);
}
