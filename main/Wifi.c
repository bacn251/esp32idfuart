#include "Wifi.h"
#include "WifiConfig.h"
#include "esp_mac.h"

static const char *TAG = "WIFI_STA";
static const char *TAG_AP = "WIFI_AP";
static const char *TAG_BTN = "WIFI_BTN";

/* -------------------------------------------------------
 * Shared state – declared extern in Wifi.h
 * ------------------------------------------------------- */
EventGroupHandle_t s_wifi_event_group = NULL;
int isConnected = 0; /* 1 = STA got IP        */
int isAPMode = 0;    /* 1 = running as SoftAP */

/* Retry counter – chỉ dùng cho lần kết nối đầu tiên */
static int s_retry_num = 0;

/* Đã từng kết nối thành công ít nhất 1 lần chưa.
 * Sau khi true → disconnect sẽ retry vô hạn (auto-reconnect). */
static bool s_sta_connected_once = false;
static bool s_sta_frist_connecte_false = false;
/* Track whether esp_wifi_init() has been called */
static bool s_wifi_initialized = false;

/* -------------------------------------------------------
 * STA event handler
 * ------------------------------------------------------- */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        isConnected = 0;

        if (s_sta_connected_once || s_sta_frist_connecte_false)
        {
            /* ---- Đã từng kết nối → mất mạng tạm thời → retry vô hạn ---- */
            ESP_LOGW(TAG, "Disconnected. Auto-reconnecting in 1s...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_wifi_connect();
        }
        else
        {
            /* ---- Lần kết nối đầu tiên → retry có giới hạn ---- */
            if (s_retry_num < ESP_MAXIMUM_RETRY)
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying initial connection... (%d/%d)", s_retry_num,
                         ESP_MAXIMUM_RETRY);
            }
            else
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                s_sta_frist_connecte_false = true;
                ESP_LOGE(TAG, "Initial connection FAILED after %d retries.",
                         ESP_MAXIMUM_RETRY);
            }
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_sta_connected_once = true; /* bật chế độ auto-reconnect vô hạn */
        isConnected = 1;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* -------------------------------------------------------
 * SoftAP event handler
 * ------------------------------------------------------- */
static void ap_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *e =
            (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG_AP, "Client connected  – MAC: " MACSTR ", AID: %d",
                 MAC2STR(e->mac), e->aid);
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *e =
            (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG_AP, "Client disconnected – MAC: " MACSTR ", AID: %d",
                 MAC2STR(e->mac), e->aid);
    }
}

/* -------------------------------------------------------
 * wifi_driver_init()  – khởi tạo WiFi stack (1 lần duy nhất)
 * ------------------------------------------------------- */
static void wifi_driver_init(void)
{
    if (s_wifi_initialized)
        return;

    if (s_wifi_event_group == NULL)
        s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    s_wifi_initialized = true;
}

/* -------------------------------------------------------
 * wifi_start_softap()
 *   Khởi động SoftAP + HTTP config server.
 * ------------------------------------------------------- */
void wifi_start_softap(void)
{
    isAPMode = 1;

    wifi_driver_init(); /* no-op nếu đã init */

    esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_event_handler, NULL, NULL));

    wifi_config_t ap_config = {
        .ap =
            {
                .ssid = SOFTAP_SSID,
                .ssid_len = strlen(SOFTAP_SSID),
                .channel = SOFTAP_CHANNEL,
                .password = SOFTAP_PASS,
                .max_connection = SOFTAP_MAX_CONN,
                .authmode = (strlen(SOFTAP_PASS) == 0) ? WIFI_AUTH_OPEN
                                                       : WIFI_AUTH_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGW(TAG_AP, "================================================");
    ESP_LOGW(TAG_AP, " SoftAP started – cấu hình qua trình duyệt");
    ESP_LOGW(TAG_AP, " SSID     : %s", SOFTAP_SSID);
    ESP_LOGW(TAG_AP, " Password : %s",
             strlen(SOFTAP_PASS) ? SOFTAP_PASS : "(open)");
    ESP_LOGW(TAG_AP, " URL      : http://192.168.4.1");
    ESP_LOGW(TAG_AP, "================================================");

    wifi_config_server_start();
}

/* -------------------------------------------------------
 * wifi_connect_sta()
 *   Kết nối STA với SSID/pass cho trước.
 *   Trả về true nếu thành công.
 * ------------------------------------------------------- */
static bool wifi_connect_sta(const char *ssid, const char *pass)
{
    wifi_driver_init();

    esp_netif_create_default_wifi_sta();

    /* Đăng ký handler KHÔNG unregister khi thành công –
     * handler phải tồn tại suốt để xử lý auto-reconnect.   */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass,
            sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
    wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to '%s'  (max %d retries)...", ssid,
             ESP_MAXIMUM_RETRY);

    /* Chờ kết quả lần kết nối đầu tiên */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        /* Handler GIỮ NGUYÊN → tự động reconnect khi mất mạng */
        ESP_LOGI(TAG, "Connected to AP  SSID: '%s'", ssid);
        return true;
    }

    /* Kết nối thất bại – handler không còn cần thiết */
    ESP_LOGE(TAG, "Cannot connect to '%s'", ssid);
    return false;
}

/* -------------------------------------------------------
 * wifi_button_task()
 *   Background task – chạy suốt vòng đời ứng dụng.
 *
 *   Hành vi giống chuẩn product (Tasmota / ESPHome):
 *     - Poll GPIO0 mỗi AP_TRIGGER_POLL_MS ms
 *     - Nút nhấn (LOW) liên tục >= AP_TRIGGER_HOLD_MS
 *       → xóa WiFi credentials trong NVS
 *       → esp_restart()
 *       → boot lại, không có credentials → tự vào SoftAP
 *     - Nhả nút trước khi đủ thời gian → reset đếm, thử lại
 * ------------------------------------------------------- */
static void wifi_button_task(void *arg)
{
    /* Cấu hình GPIO0 input pull-up (BOOT button active-LOW) */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG_BTN,
             "Button monitor started (GPIO%d). "
             "Hold %ds to enter AP config mode.",
             BOOT_BUTTON_GPIO, AP_TRIGGER_HOLD_MS / 1000);

    int held_ms = 0;
    int last_print_sec = -1;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(AP_TRIGGER_POLL_MS));

        if (gpio_get_level(BOOT_BUTTON_GPIO) == 0)
        {
            /* Nút đang giữ */
            held_ms += AP_TRIGGER_POLL_MS;

            int held_sec = held_ms / 1000;
            if (held_sec != last_print_sec)
            {
                last_print_sec = held_sec;
                int remaining = (AP_TRIGGER_HOLD_MS / 1000) - held_sec;
                if (remaining > 0)
                {
                    ESP_LOGW(TAG_BTN,
                             "BOOT held %ds / %ds – nhả để huỷ, tiếp tục giữ để vào AP "
                             "mode...",
                             held_sec, AP_TRIGGER_HOLD_MS / 1000);
                }
            }

            if (held_ms >= AP_TRIGGER_HOLD_MS)
            {
                /* ---- Đủ thời gian → xóa NVS và restart ---- */
                ESP_LOGW(TAG_BTN,
                         ">>> Xác nhận! Xóa WiFi credentials và khởi động lại... <<<");
                wifi_nvs_clear_credentials();
                vTaskDelay(pdMS_TO_TICKS(200)); /* chờ log flush */
                esp_restart();
                /* không bao giờ đến đây */
            }
        }
        else
        {
            /* Nút nhả → reset đếm */
            if (held_ms > 0)
            {
                ESP_LOGI(TAG_BTN, "Nút nhả (%dms) – huỷ. Giữ đủ %ds để vào AP mode.",
                         held_ms, AP_TRIGGER_HOLD_MS / 1000);
                held_ms = 0;
                last_print_sec = -1;
            }
        }
    }
}

/* -------------------------------------------------------
 * wifi_button_monitor_start()
 *   Spawn background task – gọi 1 lần trong app_main()
 *   SAU khi wifi_start() trả về.
 * ------------------------------------------------------- */
void wifi_button_monitor_start(void)
{
    xTaskCreate(wifi_button_task, "wifi_btn_task",
                2048,    /* stack – đủ cho GPIO + log  */
                NULL, 5, /* priority thấp, không ảnh hưởng task chính */
                NULL);
}

/* -------------------------------------------------------
 * wifi_start()  – entry point chính, gọi từ app_main()
 *
 *  Luồng:
 *    [1] Đọc NVS:
 *          Không có → SoftAP (lần đầu cấu hình)
 *          Có       → kết nối STA
 *    [2] Kết nối STA:
 *          OK   → isConnected = 1, chạy bình thường
 *          FAIL → log lỗi, isConnected = 0
 *               → KHÔNG tự vào AP (người dùng giữ BOOT 10s để reset)
 * ------------------------------------------------------- */
void wifi_start(void)
{
    if (s_wifi_event_group == NULL)
        s_wifi_event_group = xEventGroupCreate();

    /* ---- [1] Đọc credentials từ NVS ---- */
    char nvs_ssid[33] = {0};
    char nvs_pass[65] = {0};

    esp_err_t nvs_err = wifi_nvs_load_credentials(nvs_ssid, sizeof(nvs_ssid),
                                                  nvs_pass, sizeof(nvs_pass));

    if (nvs_err != ESP_OK || strlen(nvs_ssid) == 0)
    {
        ESP_LOGW(TAG, "Chưa có WiFi credentials trong NVS.");
        ESP_LOGW(TAG, ">>> Vào AP mode để cấu hình WiFi lần đầu <<<");
        wifi_start_softap();
        return;
    }

    ESP_LOGI(TAG, "NVS credentials found → SSID='%s'", nvs_ssid);

    /* ---- [2] Kết nối STA ---- */
    bool ok = wifi_connect_sta(nvs_ssid, nvs_pass);

    if (!ok)
    {
        ESP_LOGE(TAG, "==========================================================");
        ESP_LOGE(TAG, " WiFi FAILED – không thể kết nối tới '%s'.", nvs_ssid);
        ESP_LOGE(TAG, " Thiết bị tiếp tục chạy KHÔNG có WiFi.");
        ESP_LOGE(TAG, " Để cấu hình lại: giữ nút BOOT >= %ds.",
                 AP_TRIGGER_HOLD_MS / 1000);
        ESP_LOGE(TAG, "==========================================================");
    }
}
