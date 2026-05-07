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
#include "mqtt_client.h"
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
#define MQTT_BROKER_URL "mqtt://broker.emqx.io:1883"
#define MQTT_PUB_HUM "esp32/dht/temperature"
#define MQTT_PUB_TEMP "esp32/dht/humidity"
static esp_mqtt_client_handle_t mqtt_client;
#define MQTT_CONNECTED_BIT BIT1
#define BUF_SIZE (1024)
static const char *TAG = "esp-dht-station";
// static EventGroupHandle_t s_wifi_event_group;

// ---- QoS 1 ACK tracking ------------------------------------------------
#define MQTT_PENDING_MAX 8           // max in-flight QoS-1 messages
#define MQTT_PUBACK_TIMEOUT_MS 10000 // 10 s timeout waiting for PUBACK

typedef struct
{
    int msg_id;         // -1 = slot free
    TickType_t sent_at; // tick when published
    char topic[64];     // topic copy for log
} mqtt_pending_t;

static mqtt_pending_t s_pending[MQTT_PENDING_MAX];
static SemaphoreHandle_t s_pending_mutex;

static void pending_init(void)
{
    s_pending_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < MQTT_PENDING_MAX; i++)
        s_pending[i].msg_id = -1;
}

static void pending_add(int msg_id, const char *topic)
{
    if (msg_id <= 0)
        return; // QoS 0 or error → skip
    xSemaphoreTake(s_pending_mutex, portMAX_DELAY);
    for (int i = 0; i < MQTT_PENDING_MAX; i++)
    {
        if (s_pending[i].msg_id == -1)
        {
            s_pending[i].msg_id = msg_id;
            s_pending[i].sent_at = xTaskGetTickCount();
            snprintf(s_pending[i].topic, sizeof(s_pending[i].topic), "%s", topic);
            ESP_LOGI(TAG, "[QoS1] Pending PUBACK  msg_id=%d  topic=%s", msg_id, topic);
            break;
        }
    }
    xSemaphoreGive(s_pending_mutex);
}

static void pending_ack(int msg_id)
{
    xSemaphoreTake(s_pending_mutex, portMAX_DELAY);
    for (int i = 0; i < MQTT_PENDING_MAX; i++)
    {
        if (s_pending[i].msg_id == msg_id)
        {
            uint32_t rtt_ms = (xTaskGetTickCount() - s_pending[i].sent_at) * portTICK_PERIOD_MS;
            ESP_LOGI(TAG, "[QoS1] PUBACK OK  msg_id=%d  topic=%s  rtt=%lums",
                     msg_id, s_pending[i].topic, (unsigned long)rtt_ms);
            s_pending[i].msg_id = -1;
            break;
        }
    }
    xSemaphoreGive(s_pending_mutex);
}

/* Call periodically to detect timed-out publishes */
static void pending_check_timeouts(void)
{
    TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(s_pending_mutex, portMAX_DELAY);
    for (int i = 0; i < MQTT_PENDING_MAX; i++)
    {
        if (s_pending[i].msg_id != -1)
        {
            uint32_t age_ms = (now - s_pending[i].sent_at) * portTICK_PERIOD_MS;
            if (age_ms >= MQTT_PUBACK_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "[QoS1] PUBACK TIMEOUT  msg_id=%d  topic=%s  age=%lums",
                         s_pending[i].msg_id, s_pending[i].topic, (unsigned long)age_ms);
                s_pending[i].msg_id = -1; // free slot; broker will retry on reconnect
            }
        }
    }
    xSemaphoreGive(s_pending_mutex);
}
// -------------------------------------------------------------------------

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
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        xEventGroupSetBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        xEventGroupClearBits(s_wifi_event_group, MQTT_CONNECTED_BIT);
        xEventGroupWaitBits( // Wait for WIFI to reconnect
            s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(esp_mqtt_client_reconnect(client));
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        /* Broker sent PUBACK → confirm QoS-1 delivery */
        pending_ack(event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}
void DHT_task(void *pvParameter)
{
    // setDHTgpio(DHT22_PIN);
    ESP_LOGI(TAG, "Starting DHT Task");

    char hum[10];
    char temp[10];

    while (1)
    {
        xEventGroupWaitBits( // Wait for MQTT connection
            s_wifi_event_group,
            MQTT_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        ESP_LOGI(TAG, "Reading DHT");
        // int ret = readDHT();

        // errorHandler(ret);

        sprintf(hum, "%.1f", 50.5);
        sprintf(temp, "%.1f", 60.5);
        ESP_LOGI(TAG, "Hum %s\n", hum);
        ESP_LOGI(TAG, "Tmp %s\n", temp);

        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_PUB_HUM, hum, 0, 2, 1);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "[QoS1] Publish FAILED (humidity) – broker not ready?");
        }
        else
        {
            pending_add(msg_id, MQTT_PUB_HUM);
        }

        msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_PUB_TEMP, temp, 0, 1, 0);
        if (msg_id < 0)
        {
            ESP_LOGE(TAG, "[QoS1] Publish FAILED (temperature) – broker not ready?");
        }
        else
        {
            pending_add(msg_id, MQTT_PUB_TEMP);
        }

        /* Check for any publishes that never got PUBACK */
        pending_check_timeouts();

        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
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

    if (isConnected)
    {
        ESP_LOGI(APP_TAG, "[WiFi] Mode: STA  →  connected ✓");
    }
    else if (isAPMode)
    {
        ESP_LOGW(APP_TAG, "[WiFi] Mode: SoftAP  →  SSID: \"%s\"  IP: 192.168.4.1", SOFTAP_SSID);
        ESP_LOGW(APP_TAG, "[WiFi] Access http://192.168.4.1 to configure WiFi.");
    }
    else
    {
        ESP_LOGE(APP_TAG, "[WiFi] WiFi connection failed – hold BOOT >= 10s to reconfigure.");
    }

    /* --- Start background task monitoring BOOT button ---
     *  Hold button >= 10s → clear NVS WiFi → restart → enter SoftAP
     * ---------------------------------------------------- */
    wifi_button_monitor_start();
    pending_init();
    mqtt_app_start();
    uart_init_config(&uart_gps, GPS_UART_NUM, GPS_UART_BAUD_RATE, GPS_TXD_PIN, GPS_RXD_PIN, UART_ROLE_GPS);
    uart_init_config(&uart_modbus, MODBUS_UART_NUM, MODBUS_UART_BAUD_RATE, MODBUS_TXD_PIN, MODBUS_RXD_PIN, UART_ROLE_MODBUS);
    xTaskCreate(uart_event_task, "uart_gps_event_task", 3072, &uart_gps, 12, NULL);
    xTaskCreate(uart_process_task, "uart_gps_process_task", 8192, &uart_gps, 11, NULL);

    xTaskCreate(uart_event_task, "uart_modbus_event_task", 3072, &uart_modbus, 12, NULL);
    xTaskCreate(uart_process_task, "uart_modbus_process_task", 8192, &uart_modbus, 11, NULL);

    xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
}
{id : '826', character : '于是', pinyin : 'yúshì', meaning_vn : 'thế là, cho nên', meaning_en : 'consequently, hence', category : 'Conjunction', lesson : 6, level : 3},
    {id : '827', character : '与', pinyin : 'yǔ', meaning_vn : 'và, với', meaning_en : 'and, with', category : 'Conjunction', lesson : 6, level : 3},
    {id : '828', character : '语法', pinyin : 'yǔfǎ', meaning_vn : 'ngữ pháp', meaning_en : 'grammar', category : 'Noun', lesson : 6, level : 3},
    {id : '829', character : '语言', pinyin : 'yǔyán', meaning_vn : 'ngôn ngữ', meaning_en : 'language', category : 'Noun', lesson : 6, level : 3},
    {id : '830', character : '圆', pinyin : 'yuán', meaning_vn : 'tròn', meaning_en : 'round, circular', category : 'Adjective', lesson : 7, level : 3},
    {id : '831', character : '原来', pinyin : 'yuánlái', meaning_vn : 'vốn dĩ, hóa ra', meaning_en : 'original, formerly', category : 'Adjective', lesson : 7, level : 3},
    {id : '832', character : '原谅', pinyin : 'yuánliàng', meaning_vn : 'tha thứ, bỏ qua', meaning_en : 'to excuse, to forgive', category : 'Verb', lesson : 7, level : 3},
    {id : '833', character : '原因', pinyin : 'yuányīn', meaning_vn : 'nguyên nhân', meaning_en : 'cause, reason', category : 'Noun', lesson : 7, level : 3},
    {id : '834', character : '愿望', pinyin : 'yuànwàng', meaning_vn : 'nguyện vọng, mong muốn', meaning_en : 'desire, wish', category : 'Noun', lesson : 7, level : 3},
    {id : '835', character : '愿意', pinyin : 'yuànyì', meaning_vn : 'bằng lòng, muốn', meaning_en : 'to be willing', category : 'Verb', lesson : 7, level : 3},
    {id : '836', character : '约会', pinyin : 'yuēhuì', meaning_vn : 'hẹn hò, cuộc hẹn', meaning_en : 'appointment, date', category : 'Noun', lesson : 7, level : 3},
    {id : '837', character : '阅读', pinyin : 'yuèdú', meaning_vn : 'đọc hiểu', meaning_en : 'to read, reading', category : 'Verb', lesson : 7, level : 3},
    {id : '838', character : '月亮', pinyin : 'yuèliang', meaning_vn : 'mặt trăng', meaning_en : 'moon', category : 'Noun', lesson : 8, level : 3},
    {id : '839', character : '越...越...', pinyin : 'yuè...yuè...', meaning_vn : 'càng... càng...', meaning_en : 'the more... the more...', category : 'Adverb', lesson : 8, level : 3},
    {id : '840', character : '云', pinyin : 'yún', meaning_vn : 'mây', meaning_en : 'cloud', category : 'Noun', lesson : 8, level : 3},
    {id : '841', character : '允许', pinyin : 'yǔnxǔ', meaning_vn : 'cho phép', meaning_en : 'to permit, to allow', category : 'Verb', lesson : 8, level : 3},
    {id : '842', character : '杂志', pinyin : 'zázhì', meaning_vn : 'tạp chí', meaning_en : 'magazine', category : 'Noun', lesson : 8, level : 3},
    {id : '843', character : '咱们', pinyin : 'zánmen', meaning_vn : 'chúng ta (bao gồm người nghe)', meaning_en : 'we, us', category : 'Pronoun', lesson : 8, level : 3},
    {id : '844', character : '暂时', pinyin : 'zànshí', meaning_vn : 'tạm thời', meaning_en : 'temporary', category : 'Adjective', lesson : 8, level : 3},
    {id : '845', character : '脏', pinyin : 'zāng', meaning_vn : 'bẩn, dơ', meaning_en : 'dirty, filthy', category : 'Adjective', lesson : 8, level : 3},
    {id : '846', character : '责任', pinyin : 'zérèn', meaning_vn : 'trách nhiệm', meaning_en : 'responsibility', category : 'Noun', lesson : 9, level : 3},
    {id : '847', character : '增加', pinyin : 'zēngjiā', meaning_vn : 'tăng thêm', meaning_en : 'to increase, to add', category : 'Verb', lesson : 9, level : 3},
    {id : '848', character : '增长', pinyin : 'zēngzhǎng', meaning_vn : 'tăng trưởng', meaning_en : 'to grow, to increase', category : 'Verb', lesson : 9, level : 3},
    {id : '849', character : '窄', pinyin : 'zhǎi', meaning_vn : 'hẹp', meaning_en : 'narrow', category : 'Adjective', lesson : 9, level : 3},
    {id : '850', character : '站', pinyin : 'zhàn', meaning_vn : 'đứng, trạm/nhà ga', meaning_en : 'to stand, station', category : 'Verb', lesson : 9, level : 3},
    {id : '851', character : '张', pinyin : 'zhāng', meaning_vn : 'tờ, tấm (lượng từ)', meaning_en : 'measure word (flat objects)', category : 'Measure Word', lesson : 10, level : 3},
    {id : '852', character : '着急', pinyin : 'zháojí', meaning_vn : 'sốt ruột, lo lắng', meaning_en : 'worried, anxious', category : 'Adjective', lesson : 10, level : 3},
    {id : '853', character : '找', pinyin : 'zhǎo', meaning_vn : 'tìm kiếm', meaning_en : 'to look for', category : 'Verb', lesson : 10, level : 3},
    {id : '854', character : '照顾', pinyin : 'zhàogù', meaning_vn : 'chăm sóc', meaning_en : 'to take care of', category : 'Verb', lesson : 10, level : 3},
    {id : '855', character : '照片', pinyin : 'zhàopiàn', meaning_vn : 'bức ảnh', meaning_en : 'picture, photo', category : 'Noun', lesson : 10, level : 3},
    {id : '856', character : '照相机', pinyin : 'zhàoxiàngjī', meaning_vn : 'máy ảnh', meaning_en : 'camera', category : 'Noun', lesson : 10, level : 3},
    {id : '857', character : '这', pinyin : 'zhè', meaning_vn : 'đây, này', meaning_en : 'this', category : 'Pronoun', lesson : 10, level : 1},
    {id : '858', character : '着', pinyin : 'zhe', meaning_vn : 'đang (trợ từ động thái)', meaning_en : 'aspect particle', category : 'Particle', lesson : 10, level : 2},
    {id : '859', character : '真', pinyin : 'zhēn', meaning_vn : 'thật, thực sự', meaning_en : 'really, truly', category : 'Adverb', lesson : 11, level : 2},
    {id : '860', character : '真正', pinyin : 'zhēnzhèng', meaning_vn : 'chân chính, thực sự', meaning_en : 'genuine, real', category : 'Adjective', lesson : 11, level : 3},
    {id : '861', character : '整理', pinyin : 'zhěnglǐ', meaning_vn : 'sắp xếp, chỉnh đốn', meaning_en : 'to put in order', category : 'Verb', lesson : 11, level : 3},
    {id : '862', character : '整齐', pinyin : 'zhěngqí', meaning_vn : 'gọn gàng, ngăn nắp', meaning_en : 'tidy, neat', category : 'Adjective', lesson : 11, level : 3},
    {id : '863', character : '正常', pinyin : 'zhèngcháng', meaning_vn : 'bình thường', meaning_en : 'normal, regular', category : 'Adjective', lesson : 11, level : 3},
    {id : '864', character : '正好', pinyin : 'zhènghǎo', meaning_vn : 'vừa vặn, đúng lúc', meaning_en : 'just right, just in time', category : 'Adjective', lesson : 11, level : 3},
    {id : '865', character : '正确', pinyin : 'zhèngquè', meaning_vn : 'chính xác, đúng đắn', meaning_en : 'correct, right', category : 'Adjective', lesson : 11, level : 3},
    {id : '866', character : '正式', pinyin : 'zhèngshì', meaning_vn : 'chính thức', meaning_en : 'formal, official', category : 'Adjective', lesson : 11, level : 3},
    {id : '867', character : '证明', pinyin : 'zhèngmíng', meaning_vn : 'chứng minh', meaning_en : 'to prove, certificate', category : 'Verb', lesson : 12, level : 3},
    {id : '868', character : '之', pinyin : 'zhī', meaning_vn : 'của (tương đương 的)', meaning_en : 'possessive particle', category : 'Particle', lesson : 12, level : 3},
    {id : '869', character : '只', pinyin : 'zhī', meaning_vn : 'con (lượng từ động vật)', meaning_en : 'measure word (animals)', category : 'Measure Word', lesson : 12, level : 2},
    {id : '870', character : '支持', pinyin : 'zhīchí', meaning_vn : 'ủng hộ', meaning_en : 'to support', category : 'Verb', lesson : 12, level : 3},
    {id : '871', character : '知道', pinyin : 'zhīdào', meaning_vn : 'biết', meaning_en : 'to know', category : 'Verb', lesson : 12, level : 1},
    {id : '872', character : '知识', pinyin : 'zhīshi', meaning_vn : 'kiến thức', meaning_en : 'knowledge', category : 'Noun', lesson : 12, level : 3},
    {id : '873', character : '直接', pinyin : 'zhíjiē', meaning_vn : 'trực tiếp', meaning_en : 'direct, immediate', category : 'Adjective', lesson : 12, level : 3},
    {id : '874', character : '值得', pinyin : 'zhídé', meaning_vn : 'đáng giá, xứng đáng', meaning_en : 'to be worth', category : 'Verb', lesson : 12, level : 3},
    {id : '875', character : '职业', pinyin : 'zhíyè', meaning_vn : 'nghề nghiệp', meaning_en : 'occupation, profession', category : 'Noun', lesson : 13, level : 3},
    {id : '876', character : '指', pinyin : 'zhǐ', meaning_vn : 'chỉ, hướng vào', meaning_en : 'to point to, refer to', category : 'Verb', lesson : 13, level : 3},
    {id : '877', character : '只', pinyin : 'zhǐ', meaning_vn : 'chỉ, chỉ có', meaning_en : 'only', category : 'Adverb', lesson : 13, level : 2},
    {id : '878', character : '只要', pinyin : 'zhǐyào', meaning_vn : 'chỉ cần', meaning_en : 'as long as', category : 'Conjunction', lesson : 13, level : 3},
    {id : '879', character : '质量', pinyin : 'zhìliàng', meaning_vn : 'chất lượng', meaning_en : 'quality', category : 'Noun', lesson : 13, level : 3},
    {id : '880', character : '至少', pinyin : 'zhìshǎo', meaning_vn : 'ít nhất', meaning_en : 'at least', category : 'Adverb', lesson : 13, level : 3},
    {id : '881', character : '制造', pinyin : 'zhìzào', meaning_vn : 'chế tạo, sản xuất', meaning_en : 'to manufacture', category : 'Verb', lesson : 13, level : 3},
    {id : '882', character : '中国', pinyin : 'zhōngguó', meaning_vn : 'Trung Quốc', meaning_en : 'China', category : 'Noun', lesson : 1, level : 1},
    {id : '883', character : '中间', pinyin : 'zhōngjiān', meaning_vn : 'ở giữa', meaning_en : 'middle, center', category : 'Noun', lesson : 14, level : 2},
    {id : '884', character : '重点', pinyin : 'zhòngdiǎn', meaning_vn : 'trọng điểm', meaning_en : 'key point, focus', category : 'Noun', lesson : 14, level : 3},
    {id : '885', character : '重视', pinyin : 'zhòngshì', meaning_vn : 'coi trọng', meaning_en : 'to value, to attach importance to', category : 'Verb', lesson : 14, level : 3},
    {id : '886', character : '重要', pinyin : 'zhòngyào', meaning_vn : 'quan trọng', meaning_en : 'important', category : 'Adjective', lesson : 14, level : 2},
    {id : '887', character : '周围', pinyin : 'zhōuwéi', meaning_vn : 'xung quanh', meaning_en : 'around, surroundings', category : 'Noun', lesson : 14, level : 3},
    {id : '888', character : '猪', pinyin : 'zhū', meaning_vn : 'con lợn', meaning_en : 'pig', category : 'Noun', lesson : 14, level : 3},
    {id : '889', character : '逐渐', pinyin : 'zhújiàn', meaning_vn : 'dần dần', meaning_en : 'gradually', category : 'Adverb', lesson : 14, level : 3},
    {id : '890', character : '主动', pinyin : 'zhǔdòng', meaning_vn : 'chủ động', meaning_en : 'proactive, active', category : 'Adjective', lesson : 15, level : 3},
    {id : '891', character : '主要', pinyin : 'zhǔyào', meaning_vn : 'chủ yếu', meaning_en : 'main, principal', category : 'Adjective', lesson : 15, level : 2},
    {id : '892', character : '主意', pinyin : 'zhǔyi', meaning_vn : 'ý kiến, ý tưởng', meaning_en : 'idea, plan', category : 'Noun', lesson : 15, level : 3},
    {id : '893', character : '祝', pinyin : 'zhù', meaning_vn : 'chúc, chúc mừng', meaning_en : 'to wish', category : 'Verb', lesson : 15, level : 2},
    {id : '894', character : '注意', pinyin : 'zhùyì', meaning_vn : 'chú ý', meaning_en : 'to pay attention to', category : 'Verb', lesson : 15, level : 2},
    {id : '895', character : '著名', pinyin : 'zhùmíng', meaning_vn : 'nổi tiếng', meaning_en : 'famous, well-known', category : 'Adjective', lesson : 15, level : 3},
    {id : '896', character : '准时', pinyin : 'zhǔnshí', meaning_vn : 'đúng giờ', meaning_en : 'on time, punctual', category : 'Adjective', lesson : 15, level : 3},
    {id : '897', character : '准备', pinyin : 'zhǔnbèi', meaning_vn : 'chuẩn bị', meaning_en : 'to prepare', category : 'Verb', lesson : 16, level : 2},
    {id : '898', character : '桌子', pinyin : 'zhuōzi', meaning_vn : 'cái bàn', meaning_en : 'table, desk', category : 'Noun', lesson : 16, level : 1},
    {id : '899', character : '仔细', pinyin : 'zǐxì', meaning_vn : 'tỉ mỉ, cẩn thận', meaning_en : 'careful, attentive', category : 'Adjective', lesson : 16, level : 3},
    {id : '900', character : '字', pinyin : 'zì', meaning_vn : 'chữ', meaning_en : 'character, word', category : 'Noun', lesson : 16, level : 1},
    {id : '901', character : '字典', pinyin : 'zìdiǎn', meaning_vn : 'tự điển', meaning_en : 'dictionary', category : 'Noun', lesson : 17, level : 3},
    {id : '902', character : '自己', pinyin : 'zìjǐ', meaning_vn : 'tự mình', meaning_en : 'self', category : 'Pronoun', lesson : 17, level : 2}, {id : '903', character : '自行车', pinyin : 'zìxíngchē', meaning_vn : 'xe đạp', meaning_en : 'bicycle', category : 'Noun', lesson : 17, level : 2}, {id : '904', character : '自然', pinyin : 'zìrán', meaning_vn : 'tự nhiên', meaning_en : 'nature, natural', category : 'Adjective', lesson : 17, level : 3}, {id : '905', character : '自满', pinyin : 'zìmǎn', meaning_vn : 'tự mãn', meaning_en : 'complacent', category : 'Adjective', lesson : 17, level : 3}, {id : '906', character : '总是', pinyin : 'zǒngshì', meaning_vn : 'luôn luôn', meaning_en : 'always', category : 'Adverb', lesson : 17, level : 2}, {id : '907', character : '总结', pinyin : 'zǒngjié', meaning_vn : 'tổng kết', meaning_en : 'to summarize', category : 'Verb', lesson : 17, level : 3}, {id : '908', character : '最近', pinyin : 'zuìjìn', meaning_vn : 'gần đây', meaning_en : 'recently', category : 'Noun', lesson : 17, level : 2}, {id : '909', character : '最后', pinyin : 'zuìhòu', meaning_vn : 'cuối cùng', meaning_en : 'finally, last', category : 'Noun', lesson : 18, level : 2}, {id : '910', character : '最好', pinyin : 'zuìhǎo', meaning_vn : 'tốt nhất', meaning_en : 'best, had better', category : 'Adverb', lesson : 18, level : 3}, {id : '911', character : '尊重', pinyin : 'zūnzhòng', meaning_vn : 'tôn trọng', meaning_en : 'to respect', category : 'Verb', lesson : 18, level : 3}, {id : '912', character : '遵守', pinyin : 'zūnshǒu', meaning_vn : 'tuân thủ', meaning_en : 'to abide by', category : 'Verb', lesson : 18, level : 3}, {id : '913', character : '左右', pinyin : 'zuǒyòu', meaning_vn : 'khoảng chừng, trái phải', meaning_en : 'around, left and right', category : 'Noun', lesson : 18, level : 2}, {id : '914', character : '作家', pinyin : 'zuòjiā', meaning_vn : 'tác giả, nhà văn', meaning_en : 'writer, author', category : 'Noun', lesson : 18, level : 3}, {id : '915', character : '作用', pinyin : 'zuòyòng', meaning_vn : 'tác dụng', meaning_en : 'action, effect', category : 'Noun', lesson : 18, level : 3}, {id : '916', character : '作者', pinyin : 'zuòzhě', meaning_vn : 'tác giả', meaning_en : 'author', category : 'Noun', lesson : 18, level : 3}, {id : '917', character : '座', pinyin : 'zuò', meaning_vn : 'tòa, hòn (lượng từ vật lớn)', meaning_en : 'measure word (buildings, mountains)', category : 'Measure Word', lesson : 19, level : 3}, {id : '918', character : '座位', pinyin : 'zuòwèi', meaning_vn : 'chỗ ngồi', meaning_en : 'seat', category : 'Noun', lesson : 19, level : 3}, {id : '919', character : '做', pinyin : 'zuò', meaning_vn : 'làm', meaning_en : 'to do, to make', category : 'Verb', lesson : 1, level : 1}, {id : '920', character : '做生意', pinyin : 'zuò shēngyì', meaning_vn : 'làm kinh doanh', meaning_en : 'to do business', category : 'Verb', lesson : 19, level : 3}, {id : '921', character : '作业', pinyin : 'zuòyè', meaning_vn : 'bài tập về nhà', meaning_en : 'homework', category : 'Noun', lesson : 19, level : 2}, {id : '922', character : '哎', pinyin : 'āi', meaning_vn : 'này, ôi (thán từ)', meaning_en : 'exclamation (hey, ah)', category : 'Particle', lesson : 19, level : 3}, {id : '923', character : '唉', pinyin : 'ài', meaning_vn : 'ôi, chao ôi', meaning_en : 'sigh, alas', category : 'Particle', lesson : 19, level : 3}, {id : '924', character : '爱', pinyin : 'ài', meaning_vn : 'yêu', meaning_en : 'to love', category : 'Verb', lesson : 1, level : 1}, {id : '925', character : '爱好', pinyin : 'àihào', meaning_vn : 'sở thích', meaning_en : 'hobby', category : 'Noun', lesson : 20, level : 3}, {id : '926', character : '爱护', pinyin : 'àihù', meaning_vn : 'yêu quý, bảo vệ', meaning_en : 'to cherish, to take care of', category : 'Verb', lesson : 20, level : 3}, {id : '927', character : '爱情', pinyin : 'àiqíng', meaning_vn : 'tình yêu', meaning_en : 'love (romantic)', category : 'Noun', lesson : 20, level : 3}, {id : '928', character : '爱心', pinyin : 'àixīn', meaning_vn : 'lòng yêu thương', meaning_en : 'compassion, love', category : 'Noun', lesson : 20, level : 3}, {id : '929', character : '安静', pinyin : 'ānjìng', meaning_vn : 'yên tĩnh', meaning_en : 'quiet, peaceful', category : 'Adjective', lesson : 20, level : 3}, {id : '930', character : '安排', pinyin : 'ānpái', meaning_vn : 'sắp xếp', meaning_en : 'to arrange', category : 'Verb', lesson : 20, level : 3}, {id : '931', character : '安全', pinyin : 'ānquán', meaning_vn : 'an toàn', meaning_en : 'safe, safety', category : 'Adjective', lesson : 20, level : 3}, {id : '932', character : '安慰', pinyin : 'ānwèi', meaning_vn : 'an ủi', meaning_en : 'to comfort', category : 'Verb', lesson : 21, level : 3}, {id : '933', character : '安装', pinyin : 'ānzhuāng', meaning_vn : 'lắp đặt', meaning_en : 'to install', category : 'Verb', lesson : 21, level : 3}, {id : '934', character : '岸', pinyin : 'àn', meaning_vn : 'bờ (sông, biển)', meaning_en : 'bank, shore', category : 'Noun', lesson : 21, level : 3}, {id : '935', character : '暗', pinyin : 'àn', meaning_vn : 'tối, ám muội', meaning_en : 'dark, dim', category : 'Adjective', lesson : 21, level : 3}, {id : '936', character : '按', pinyin : 'àn', meaning_vn : 'ấn, bấm', meaning_en : 'to press, to push', category : 'Verb', lesson : 21, level : 3}, {id : '937', character : '按照', pinyin : 'ànzhào', meaning_vn : 'dựa theo', meaning_en : 'according to', category : 'Preposition', lesson : 21, level : 3}, {id : '938', character : '八', pinyin : 'bā', meaning_vn : 'tám', meaning_en : 'eight', category : 'Number', lesson : 1, level : 1}, {id : '939', character : '把', pinyin : 'bǎ', meaning_vn : 'đem, cầm (trợ từ)', meaning_en : 'structural particle / to hold', category : 'Preposition', lesson : 22, level : 3}, {id : '940', character : '把握', pinyin : 'bǎwò', meaning_vn : 'nắm bắt, sự chắc chắn', meaning_en : 'to grasp, certainty', category : 'Verb', lesson : 22, level : 3}, {id : '941', character : '爸爸', pinyin : 'bàba', meaning_vn : 'bố, cha', meaning_en : 'dad', category : 'Noun', lesson : 1, level : 1}, {id : '942', character : '吧', pinyin : 'ba', meaning_vn : 'nhé, nha (trợ từ cuối câu)', meaning_en : 'sentence-final particle', category : 'Particle', lesson : 1, level : 1}, {id : '943', character : '白', pinyin : 'bái', meaning_vn : 'trắng', meaning_en : 'white', category : 'Adjective', lesson : 22, level : 2}, {id : '944', character : '白菜', pinyin : 'báicài', meaning_vn : 'cải thảo', meaning_en : 'Chinese cabbage', category : 'Noun', lesson : 22, level : 3}, {id : '945', character : '白天', pinyin : 'báitiān', meaning_vn : 'ban ngày', meaning_en : 'daytime', category : 'Noun', lesson : 22, level : 3}, {id : '946', character : '百', pinyin : 'bǎi', meaning_vn : 'trăm', meaning_en : 'hundred', category : 'Number', lesson : 2, level : 2}, {id : '947', character : '百分之', pinyin : 'bǎi fēn zhī', meaning_vn : 'phần trăm', meaning_en : 'percent', category : 'Number', lesson : 23, level : 3}, {id : '948', character : '摆', pinyin : 'bǎi', meaning_vn : 'bày biện, sắp xếp', meaning_en : 'to put, to place', category : 'Verb', lesson : 23, level : 3}, {id : '949', character : '班', pinyin : 'bān', meaning_vn : 'lớp học, ca làm việc', meaning_en : 'class, shift', category : 'Noun', lesson : 23, level : 2}, {id : '950', character : '搬', pinyin : 'bān', meaning_vn : 'dọn (nhà), bê vác', meaning_en : 'to move (objects)', category : 'Verb', lesson : 23, level : 3},