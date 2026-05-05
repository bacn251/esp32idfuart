idf_component_register(
    SRCS
        "WifiConfig.c"
        "Wifi.c"
        "ParseModbus.c"
        "ParseGps.c"
        "main.c"
    PRIV_REQUIRES
        esp_wifi
        nvs_flash
        esp_http_server
        driver
        esp_timer
        esp_event
        esp_netif
        lwip
    INCLUDE_DIRS "."
)
