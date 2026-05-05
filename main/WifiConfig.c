#include "WifiConfig.h"

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

#define NVS_NAMESPACE "wifi_cfg"
#define NVS_KEY_SSID "ssid"
#define NVS_KEY_PASS "pass"
#define TAG_CFG "WIFI_CFG"

static httpd_handle_t s_server = NULL;

/* -------------------------------------------------------
 * HTML pages (embedded)
 * ------------------------------------------------------- */
static const char *HTML_CONFIG =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 WiFi Config</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:'Segoe UI',sans-serif;background:#0f0c29;"
    "background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);"
    "min-height:100vh;display:flex;align-items:center;justify-content:center}"
    ".card{background:rgba(255,255,255,.07);backdrop-filter:blur(12px);"
    "border:1px solid rgba(255,255,255,.15);border-radius:20px;"
    "padding:36px 32px;width:360px;box-shadow:0 20px 60px rgba(0,0,0,.5)}"
    ".logo{text-align:center;font-size:2.6em;margin-bottom:6px}"
    "h1{text-align:center;font-size:1.25em;font-weight:700;color:#fff;"
    "margin-bottom:4px}"
    ".sub{text-align:center;font-size:.82em;color:#aaa;margin-bottom:28px}"
    "label{display:block;font-size:.8em;font-weight:600;color:#ccc;margin-bottom:6px}"
    "input{width:100%;padding:11px 14px;border-radius:10px;"
    "border:1px solid rgba(255,255,255,.15);background:rgba(255,255,255,.08);"
    "color:#fff;font-size:.95em;margin-bottom:18px;outline:none;transition:.2s}"
    "input:focus{border-color:#7c6df0;background:rgba(124,109,240,.15)}"
    "input::placeholder{color:#666}"
    "button{width:100%;padding:13px;border:none;border-radius:10px;"
    "background:linear-gradient(135deg,#7c6df0,#e94560);"
    "color:#fff;font-size:1em;font-weight:700;cursor:pointer;transition:.2s}"
    "button:hover{opacity:.85;transform:translateY(-1px)}"
    ".tip{text-align:center;font-size:.78em;color:#888;margin-top:16px}"
    "</style></head>"
    "<body><div class='card'>"
    "<div class='logo'>&#x1F4F6;</div>"
    "<h1>ESP32 Wi-Fi Setup</h1>"
    "<p class='sub'>Nhập thông tin mạng WiFi để kết nối</p>"
    "<form method='POST' action='/save'>"
    "<label>&#x1F4E1; Tên mạng (SSID)</label>"
    "<input type='text'     name='ssid' placeholder='VD: MyHomeWiFi'  required maxlength='32'>"
    "<label>&#x1F512; Mật khẩu</label>"
    "<input type='password' name='pass' placeholder='Mật khẩu WiFi'           maxlength='64'>"
    "<button type='submit'>&#x1F4BE; Lưu &amp; Kết nối</button>"
    "</form>"
    "<p class='tip'>Thiết bị sẽ tự động khởi động lại sau khi lưu.</p>"
    "</div></body></html>";

static const char *HTML_SAVED =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Đã lưu</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:'Segoe UI',sans-serif;background:#0f0c29;"
    "background:linear-gradient(135deg,#0f0c29,#302b63,#24243e);"
    "min-height:100vh;display:flex;align-items:center;justify-content:center}"
    ".card{background:rgba(255,255,255,.07);backdrop-filter:blur(12px);"
    "border:1px solid rgba(255,255,255,.15);border-radius:20px;"
    "padding:40px 32px;width:340px;text-align:center;"
    "box-shadow:0 20px 60px rgba(0,0,0,.5)}"
    ".icon{font-size:3.5em;margin-bottom:14px}"
    "h2{color:#4caf50;font-size:1.4em;margin-bottom:10px}"
    "p{color:#aaa;font-size:.9em;line-height:1.6}"
    ".dot{display:inline-block;width:8px;height:8px;border-radius:50%;"
    "background:#4caf50;animation:pulse 1s infinite}"
    "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}"
    "</style></head>"
    "<body><div class='card'>"
    "<div class='icon'>&#x2705;</div>"
    "<h2>Đã lưu thành công!</h2>"
    "<p>ESP32 đang khởi động lại...<br>"
    "Vui lòng chờ <span class='dot'></span></p>"
    "<p style='margin-top:14px;font-size:.8em;color:#666'>"
    "Kết nối lại WiFi của bạn sau vài giây.</p>"
    "</div></body></html>";

/* -------------------------------------------------------
 * URL decode  (xử lý %XX và + → space)
 * ------------------------------------------------------- */
static void url_decode(char *dst, const char *src, size_t max_len)
{
    size_t i = 0;
    while (*src && i < max_len - 1)
    {
        if (*src == '+')
        {
            dst[i++] = ' ';
            src++;
        }
        else if (*src == '%' && src[1] && src[2])
        {
            char hex[3] = {src[1], src[2], '\0'};
            dst[i++] = (char)strtol(hex, NULL, 16);
            src += 3;
        }
        else
        {
            dst[i++] = *src++;
        }
    }
    dst[i] = '\0';
}

/* -------------------------------------------------------
 * GET /  →  trang nhập SSID/pass
 * ------------------------------------------------------- */
static esp_err_t get_root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, HTML_CONFIG, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* -------------------------------------------------------
 * POST /save  →  parse form, lưu NVS, restart
 * ------------------------------------------------------- */
static esp_err_t post_save_handler(httpd_req_t *req)
{
    char body[256] = {0};
    int recv_len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (recv_len <= 0)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    body[recv_len] = '\0';
    ESP_LOGI(TAG_CFG, "POST body: %s", body);

    /* Parse URL-encoded: ssid=xxx&pass=yyy */
    char raw_ssid[65] = {0};
    char raw_pass[65] = {0};

    char *p = strstr(body, "ssid=");
    if (p)
    {
        p += 5;
        char *end = strchr(p, '&');
        size_t len = end ? (size_t)(end - p) : strlen(p);
        if (len >= sizeof(raw_ssid))
            len = sizeof(raw_ssid) - 1;
        strncpy(raw_ssid, p, len);
    }

    p = strstr(body, "pass=");
    if (p)
    {
        p += 5;
        char *end = strchr(p, '&');
        size_t len = end ? (size_t)(end - p) : strlen(p);
        if (len >= sizeof(raw_pass))
            len = sizeof(raw_pass) - 1;
        strncpy(raw_pass, p, len);
    }

    /* URL decode */
    char ssid[33] = {0};
    char pass[65] = {0};
    url_decode(ssid, raw_ssid, sizeof(ssid));
    url_decode(pass, raw_pass, sizeof(pass));

    if (strlen(ssid) == 0)
    {
        const char *err_msg = "SSID khong duoc de trong!";
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, err_msg, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_CFG, "Saving credentials → SSID: '%s'", ssid);
    wifi_nvs_save_credentials(ssid, pass);

    /* Gửi trang thành công */
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, HTML_SAVED, HTTPD_RESP_USE_STRLEN);

    /* Restart sau 2 giây để trình duyệt nhận response */
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return ESP_OK;
}

/* -------------------------------------------------------
 * wifi_config_server_start()
 * ------------------------------------------------------- */
void wifi_config_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG_CFG, "Failed to start HTTP server!");
        return;
    }

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_root_handler,
    };
    httpd_register_uri_handler(s_server, &uri_root);

    httpd_uri_t uri_save = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = post_save_handler,
    };
    httpd_register_uri_handler(s_server, &uri_save);

    ESP_LOGI(TAG_CFG, "Config server ready → http://192.168.4.1");
}

/* -------------------------------------------------------
 * wifi_config_server_stop()
 * ------------------------------------------------------- */
void wifi_config_server_stop(void)
{
    if (s_server)
    {
        httpd_stop(s_server);
        s_server = NULL;
    }
}

/* -------------------------------------------------------
 * wifi_nvs_save_credentials()
 * ------------------------------------------------------- */
esp_err_t wifi_nvs_save_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_CFG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_str(h, NVS_KEY_SSID, ssid);
    nvs_set_str(h, NVS_KEY_PASS, pass ? pass : "");
    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG_CFG, "NVS saved → SSID='%s'", ssid);
    }
    else
    {
        ESP_LOGE(TAG_CFG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    return err;
}

/* -------------------------------------------------------
 * wifi_nvs_load_credentials()
 * ------------------------------------------------------- */
esp_err_t wifi_nvs_load_credentials(char *ssid, size_t ssid_len,
                                    char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK)
        return err; /* chưa có namespace → chưa lưu lần nào */

    err = nvs_get_str(h, NVS_KEY_SSID, ssid, &ssid_len);
    if (err != ESP_OK)
    {
        nvs_close(h);
        return err;
    }

    err = nvs_get_str(h, NVS_KEY_PASS, pass, &pass_len);
    nvs_close(h);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG_CFG, "NVS loaded → SSID='%s'", ssid);
    }
    return err;
}

/* -------------------------------------------------------
 * wifi_nvs_clear_credentials()
 * ------------------------------------------------------- */
void wifi_nvs_clear_credentials(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK)
    {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGW(TAG_CFG, "NVS WiFi credentials cleared!");
    }
}
