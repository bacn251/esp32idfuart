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

static const int RX_BUF_SIZE = 128;
static const char *APP_TAG = "APP_MAIN";

// Get values from Kconfig
#define APP_VERSION CONFIG_MY_APP_VERSION
#define LED_NUM CONFIG_MY_APP_LED_GPIO
#define LED_ACTIVE_LEVEL CONFIG_MY_APP_LED_ACTIVE_LEVEL
#define UART_NUM CONFIG_MY_APP_UART_PORT
#define UART_BAUD_RATE CONFIG_MY_APP_UART_BAUD_RATE
#define TXD_PIN CONFIG_MY_APP_UART_TX_PIN
#define RXD_PIN CONFIG_MY_APP_UART_RX_PIN

TaskHandle_t txTaskHandle;
uint8_t RxData[128];

void ledConfig (void)
{
	gpio_reset_pin(LED_NUM);
	gpio_set_direction(LED_NUM, GPIO_MODE_OUTPUT);
	ESP_LOGI(APP_TAG, "LED configured on GPIO %d (Active Level: %s)", 
	         LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
}

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
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void tx_task(void *arg)
{
	uint8_t *data = (uint8_t*) malloc(150);
    while (1) {
		int len = sprintf((char *)data, "Data Received is '%s' \n", RxData);
		uart_write_bytes(UART_NUM, data, len);
		vTaskSuspend(NULL);
    }
    free(data);
}

static void rx_task(void *arg)
{
	int ledState = 0;
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM, RxData, RX_BUF_SIZE, 200 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            RxData[rxBytes] = 0;
            if (strcmp((char*)RxData, "ON") == 0) 
                ledState = LED_ACTIVE_LEVEL ? 1 : 0;
            else if (strcmp((char*)RxData, "OFF") == 0) 
                ledState = LED_ACTIVE_LEVEL ? 0 : 1;
            gpio_set_level(LED_NUM, ledState);
            vTaskResume(txTaskHandle);
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, RxData);
            ESP_LOG_BUFFER_HEX(RX_TASK_TAG, RxData, rxBytes);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(APP_TAG, "========== Application Started ==========");
    ESP_LOGI(APP_TAG, "App Version: %s", APP_VERSION);
    ESP_LOGI(APP_TAG, "LED GPIO: %d (Active: %s)", LED_NUM, LED_ACTIVE_LEVEL ? "HIGH" : "LOW");
    ESP_LOGI(APP_TAG, "UART Port: %d, Baud: %d, TX: %d, RX: %d", 
             UART_NUM, UART_BAUD_RATE, TXD_PIN, RXD_PIN);
    ESP_LOGI(APP_TAG, "==========================================");
    
    init();
    ledConfig();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, &txTaskHandle);
}
