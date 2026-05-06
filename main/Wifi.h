#ifndef WIFI_H
#define WIFI_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

/* ---------- Kconfig mappings ---------- */
#define ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* ---------- Boot-button AP trigger ----------
 *  GPIO0 là nút BOOT trên hầu hết dev-board ESP32.
 *  Giữ nút trong AP_TRIGGER_HOLD_MS ms ngay khi khởi động
 *  → xóa NVS credentials → vào AP mode để cấu hình lại WiFi.
 * ----------------------------------------- */
#define BOOT_BUTTON_GPIO       0
#define AP_TRIGGER_HOLD_MS  10000   /* 10 giây */
#define AP_TRIGGER_POLL_MS     50   /* kiểm tra mỗi 50 ms */

/* ---------- WPA3 SAE mode ---------- */
#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#else
/* Default: disable WPA3 SAE */
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_UNSPECIFIED
#define EXAMPLE_H2E_IDENTIFIER ""
#endif

/* ---------- Auth-mode threshold ---------- */
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#else
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#endif

/* ---------- SoftAP config ---------- */
#define SOFTAP_SSID       "ESP32-Config"
#define SOFTAP_PASS       "12345678"   /* min 8 chars, dùng "" để open */
#define SOFTAP_CHANNEL    1
#define SOFTAP_MAX_CONN   4

/* ---------- Event bits ---------- */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* ---------- Shared state (defined in Wifi.c) ---------- */
extern EventGroupHandle_t s_wifi_event_group;
extern int isConnected; /* 1 = STA connected     */
extern int isAPMode;    /* 1 = running as SoftAP */

/* ---------- Public API ---------- */

/**
 * @brief  Kết nối WiFi khi boot.
 *
 * Luồng:
 *  1. Đọc NVS:
 *       – Có credentials  → kết nối STA.
 *       – Không có        → vào SoftAP (cấu hình lần đầu).
 *  2. Kết nối STA:
 *       – Thành công → isConnected = 1, chạy bình thường.
 *       – Thất bại   → log lỗi, isConnected = 0.
 *         (KHÔNG tự vào AP – người dùng giữ BOOT để reset).
 */
void wifi_start(void);

/**
 * @brief  Spawn background FreeRTOS task theo dõi nút BOOT (GPIO0).
 *
 * Gọi 1 lần trong app_main() SAU wifi_start().
 * Hành vi:
 *  – Giữ nút >= AP_TRIGGER_HOLD_MS (mặc định 10s) BẤT KỲ LÚC NÀO
 *    → xóa WiFi credentials trong NVS → esp_restart()
 *    → boot lại, không có credentials → tự vào SoftAP.
 *  – Nhả trước khi đủ thời gian → huỷ, tiếp tục chạy bình thường.
 */
void wifi_button_monitor_start(void);

/** Khởi động SoftAP + HTTP config server (có thể gọi độc lập). */
void wifi_start_softap(void);

#endif /* WIFI_H */