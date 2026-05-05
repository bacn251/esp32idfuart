#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "esp_err.h"
#include <stddef.h>

/* -------------------------------------------------------
 * NVS helpers
 *   namespace : "wifi_cfg"
 *   keys      : "ssid" , "pass"
 * ------------------------------------------------------- */
esp_err_t wifi_nvs_save_credentials(const char *ssid, const char *pass);
esp_err_t wifi_nvs_load_credentials(char *ssid, size_t ssid_len,
                                    char *pass, size_t pass_len);
void wifi_nvs_clear_credentials(void);

/* -------------------------------------------------------
 * HTTP config server  (chạy khi ở chế độ SoftAP)
 *   GET  /       → trang nhập SSID / password
 *   POST /save   → lưu NVS rồi restart
 * ------------------------------------------------------- */
void wifi_config_server_start(void);
void wifi_config_server_stop(void);

#endif /* WIFI_CONFIG_H */
