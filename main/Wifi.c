#include "Wifi.h"
#include "WifiConfig.h"

static const char *TAG = "WIFI_STA";
static const char *TAG_AP = "WIFI_AP";

/* -------------------------------------------------------
 * Shared state – declared extern in Wifi.h
 * ------------------------------------------------------- */
EventGroupHandle_t s_wifi_event_group = NULL;
int isConnected = 0; /* 1 = STA got IP          */
int isAPMode = 0;    /* 1 = running as SoftAP   */

/* Retry counter (STA mode) */
static int s_retry_num = 0;

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
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying connection... (%d/%d)", s_retry_num, ESP_MAXIMUM_RETRY);
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Max retries reached. Connection FAILED.");
        }
        isConnected = 0;
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
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
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG_AP, "Client connected  – MAC: " MACSTR ", AID: %d",
                 MAC2STR(e->mac), e->aid);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG_AP, "Client disconnected – MAC: " MACSTR ", AID: %d",
                 MAC2STR(e->mac), e->aid);
    }
}

/* -------------------------------------------------------
 * wifi_start_softap()
 *   Khởi động SoftAP + HTTP config server.
 *   Có thể gọi trực tiếp (khi WiFi chưa init)
 *   hoặc gọi từ wifi_init_sta() sau khi STA fail
 *   (khi WiFi driver đã init, chỉ cần đổi mode).
 * ------------------------------------------------------- */
void wifi_start_softap(void)
{
    isAPMode = 1;

    /* Chỉ khởi tạo stack nếu chưa có (gọi trực tiếp, không qua wifi_init_sta) */
    if (!s_wifi_initialized)
    {
        if (s_wifi_event_group == NULL)
            s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        s_wifi_initialized = true;
    }

    /* Tạo netif AP */
    esp_netif_create_default_wifi_ap();

    /* Đăng ký event handler AP */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &ap_event_handler, NULL, NULL));

    /* Cấu hình SoftAP */
    wifi_config_t ap_config = {
        .ap = {
            .ssid = SOFTAP_SSID,
            .ssid_len = strlen(SOFTAP_SSID),
            .channel = SOFTAP_CHANNEL,
            .password = SOFTAP_PASS,
            .max_connection = SOFTAP_MAX_CONN,
            .authmode = (strlen(SOFTAP_PASS) == 0)
                            ? WIFI_AUTH_OPEN
                            : WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGW(TAG_AP, "================================================");
    ESP_LOGW(TAG_AP, " SoftAP started – cấu hình qua trình duyệt");
    ESP_LOGW(TAG_AP, " SSID     : %s", SOFTAP_SSID);
    ESP_LOGW(TAG_AP, " Password : %s", strlen(SOFTAP_PASS) ? SOFTAP_PASS : "(open)");
    ESP_LOGW(TAG_AP, " URL      : http://192.168.4.1");
    ESP_LOGW(TAG_AP, "================================================");

    /* Khởi động HTTP config server */
    wifi_config_server_start();
}

/* -------------------------------------------------------
 * wifi_init_sta()
 *   1. Đọc credentials từ NVS (ưu tiên)
 *   2. Fallback về Kconfig nếu NVS trống
 *   3. Kết nối STA – nếu fail → stop (KHÔNG deinit)
 *      rồi chuyển sang SoftAP mode
 * ------------------------------------------------------- */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    s_wifi_initialized = true; /* đánh dấu driver đã init */

    /* ---------- Đọc credentials ---------- */
    char nvs_ssid[33] = {0};
    char nvs_pass[65] = {0};
    const char *use_ssid;
    const char *use_pass;

    esp_err_t nvs_err = wifi_nvs_load_credentials(
        nvs_ssid, sizeof(nvs_ssid),
        nvs_pass, sizeof(nvs_pass));

    if (nvs_err == ESP_OK && strlen(nvs_ssid) > 0)
    {
        use_ssid = nvs_ssid;
        use_pass = nvs_pass;
        ESP_LOGI(TAG, "Credentials source: NVS     → SSID='%s'", use_ssid);
    }
    else
    {
        use_ssid = ESP_WIFI_SSID;
        use_pass = ESP_WIFI_PASS;
        ESP_LOGI(TAG, "Credentials source: Kconfig → SSID='%s'", use_ssid);
    }

    /* ---------- Đăng ký event handlers ---------- */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    /* ---------- Cấu hình STA ---------- */
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, use_ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, use_pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
    wifi_config.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to '%s'  (max %d retries)...", use_ssid, ESP_MAXIMUM_RETRY);

    /* ---------- Chờ kết quả ---------- */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        /* ---- Thành công ---- */
        ESP_LOGI(TAG, "Connected to AP  SSID: '%s'", use_ssid);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        /* ---- Thất bại → SoftAP fallback ---- */
        ESP_LOGE(TAG, "Cannot connect to '%s' → switching to SoftAP", use_ssid);

        /* Dừng STA – KHÔNG gọi esp_wifi_deinit()
         * để driver còn nguyên, wifi_start_softap() chỉ cần đổi mode */
        ESP_ERROR_CHECK(esp_wifi_stop());
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);
        esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);

        wifi_start_softap();
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED WiFi event");
    }
}
