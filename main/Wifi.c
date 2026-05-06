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

/* Retry counter – only for initial connection attempt */
static int s_retry_num = 0;

/* Once true → disconnect will retry indefinitely (auto-reconnect).
 * Enabled when:
 *  a) Initial connection succeeded (IP_EVENT_STA_GOT_IP)
 *  b) Initial retry exhausted → keep driver alive waiting for AP to come back */
static bool s_sta_connected_once = false;
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

        if (s_sta_connected_once)
        {
            /* ---- Infinite retry: already connected or exhausted initial retry ---- */
            ESP_LOGW(TAG, "Disconnected. Auto-reconnecting in 3s...");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_wifi_connect();
        }
        else
        {
            /* ---- Initial connection attempt, limited retry ---- */
            s_retry_num++;
            if (s_retry_num <= ESP_MAXIMUM_RETRY)
            {
                ESP_LOGI(TAG, "Retrying initial connection... (%d/%d)",
                         s_retry_num, ESP_MAXIMUM_RETRY);
                esp_wifi_connect();
            }
            else
            {
                /* Exhausted retry → report failure to wifi_connect_sta() */
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "Initial connection FAILED after %d retries.",
                         ESP_MAXIMUM_RETRY);

                /* Switch to infinite retry mode – keep driver alive
                 * waiting for AP to come back later (will auto-reconnect) */
                s_sta_connected_once = true;
                ESP_LOGW(TAG, "Switching to background retry (every 5s)...");
                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_wifi_connect();
            }
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num          = 0;
        s_sta_connected_once = true; /* enable infinite auto-reconnect mode */
        isConnected          = 1;
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
 * wifi_driver_init()  – initialize WiFi stack (only once)
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
 *   Start SoftAP + HTTP config server.
 * ------------------------------------------------------- */
void wifi_start_softap(void)
{
    isAPMode = 1;

    wifi_driver_init(); /* no-op if already initialized */

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
    ESP_LOGW(TAG_AP, " SoftAP started – configure via web browser");
    ESP_LOGW(TAG_AP, " SSID     : %s", SOFTAP_SSID);
    ESP_LOGW(TAG_AP, " Password : %s",
             strlen(SOFTAP_PASS) ? SOFTAP_PASS : "(open)");
    ESP_LOGW(TAG_AP, " URL      : http://192.168.4.1");
    ESP_LOGW(TAG_AP, "================================================");

    wifi_config_server_start();
}

/* -------------------------------------------------------
 * wifi_connect_sta()
 *   Connect STA with given SSID/password.
 *   Returns true on success.
 * ------------------------------------------------------- */
static bool wifi_connect_sta(const char *ssid, const char *pass)
{
    wifi_driver_init();

    esp_netif_create_default_wifi_sta();

    /* Register handler – do NOT unregister on success –
     * handler must persist to handle auto-reconnect.   */
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

    /* Wait for initial connection result */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        /* Handler persists → auto-reconnect on network loss */
        ESP_LOGI(TAG, "Connected to AP  SSID: '%s'", ssid);
        return true;
    }

    /* Connection failed – handler no longer needed */
    ESP_LOGE(TAG, "Cannot connect to '%s'", ssid);
    return false;
}

/* -------------------------------------------------------
 * wifi_button_task()
 *   Background task – runs for application lifetime.
 *
 *   Behavior matches standard product (Tasmota / ESPHome):
 *     - Poll GPIO0 every AP_TRIGGER_POLL_MS ms
 *     - Button pressed (LOW) continuously >= AP_TRIGGER_HOLD_MS
 *       → clear WiFi credentials in NVS
 *       → esp_restart()
 *       → reboot without credentials → automatically enter SoftAP
 *     - Release button before time elapsed → reset counter, retry
 * ------------------------------------------------------- */
static void wifi_button_task(void *arg)
{
    /* Configure GPIO0 input pull-up (BOOT button active-LOW) */
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
            /* Button is held */
            held_ms += AP_TRIGGER_POLL_MS;

            int held_sec = held_ms / 1000;
            if (held_sec != last_print_sec)
            {
                last_print_sec = held_sec;
                int remaining = (AP_TRIGGER_HOLD_MS / 1000) - held_sec;
                if (remaining > 0)
                {
                    ESP_LOGW(TAG_BTN,
                             "BOOT held %ds / %ds – release to cancel, continue holding to enter AP "
                             "mode...",
                             held_sec, AP_TRIGGER_HOLD_MS / 1000);
                }
            }

            if (held_ms >= AP_TRIGGER_HOLD_MS)
            {
                /* ---- Time reached → clear NVS and restart ---- */
                ESP_LOGW(TAG_BTN,
                         ">>> Confirmed! Clearing WiFi credentials and restarting... <<<");
                wifi_nvs_clear_credentials();
                vTaskDelay(pdMS_TO_TICKS(200)); /* wait for log flush */
                esp_restart();
                /* never reach here */
            }
        }
        else
        {
            /* Button released → reset counter */
            if (held_ms > 0)
            {
                ESP_LOGI(TAG_BTN, "Button released (%dms) – cancelled. Hold for %ds to enter AP mode.",
                         held_ms, AP_TRIGGER_HOLD_MS / 1000);
                held_ms = 0;
                last_print_sec = -1;
            }
        }
    }
}

/* -------------------------------------------------------
 * wifi_button_monitor_start()
 *   Spawn background task – call once in app_main()
 *   AFTER wifi_start() returns.
 * ------------------------------------------------------- */
void wifi_button_monitor_start(void)
{
    xTaskCreate(wifi_button_task, "wifi_btn_task",
                2048,    /* stack – sufficient for GPIO + logging  */
                NULL, 5, /* low priority, does not impact main task */
                NULL);
}

/* -------------------------------------------------------
 * wifi_start()  – main entry point, called from app_main()
 *
 *  Flow:
 *    [1] Read NVS:
 *          Not found → SoftAP (first-time configuration)
 *          Found     → connect STA
 *    [2] Connect STA:
 *          OK   → isConnected = 1, run normally
 *          FAIL → log error, isConnected = 0
 *               → Do NOT auto-enter AP (user holds BOOT 10s to reset)
 * ------------------------------------------------------- */
void wifi_start(void)
{
    if (s_wifi_event_group == NULL)
        s_wifi_event_group = xEventGroupCreate();

    /* ---- [1] Read credentials from NVS ---- */
    char nvs_ssid[33] = {0};
    char nvs_pass[65] = {0};

    esp_err_t nvs_err = wifi_nvs_load_credentials(nvs_ssid, sizeof(nvs_ssid),
                                                  nvs_pass, sizeof(nvs_pass));

    if (nvs_err != ESP_OK || strlen(nvs_ssid) == 0)
    {
        ESP_LOGW(TAG, "No WiFi credentials found in NVS.");
        ESP_LOGW(TAG, ">>> Entering AP mode for initial WiFi configuration <<<");
        wifi_start_softap();
        return;
    }

    ESP_LOGI(TAG, "NVS credentials found → SSID='%s'", nvs_ssid);

    /* ---- [2] Connect STA ---- */
    bool ok = wifi_connect_sta(nvs_ssid, nvs_pass);

    if (!ok)
    {
        ESP_LOGE(TAG, "==========================================================");
        ESP_LOGE(TAG, " WiFi FAILED – cannot connect to '%s'.", nvs_ssid);
        ESP_LOGE(TAG, " Device will continue running WITHOUT WiFi.");
        ESP_LOGE(TAG, " To reconfigure: hold BOOT button >= %ds.",
                 AP_TRIGGER_HOLD_MS / 1000);
        ESP_LOGE(TAG, "==========================================================");
    }
}
