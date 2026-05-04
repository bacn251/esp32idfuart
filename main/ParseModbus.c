#include "ParseModbus.h"
#include "esp_log.h"

static const char *TAG = "MODBUS";

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
    {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
int modbus_get_frame_len(uint8_t *buf, int len)
{
    if (len < 8)  // Minimum frame size (slave_id + func + data + crc)
        return 0;

    uint8_t slave_id = buf[0];
    uint8_t func = buf[1];
    
    // Slave ID should be 1-247
    if (slave_id < 1 || slave_id > 247)
    {
        ESP_LOGW(TAG, "Invalid slave ID: 0x%02X", slave_id);
        return 0;
    }
    
    ESP_LOGI(TAG, "Function code: 0x%02X", func);
    switch (func)
    {
    case 0x03: // Read Holding Registers REQUEST → variable length
        if (len < 8)
            return 0;
        return 8;
    case 0x06: // Write Single Register → fixed 8 bytes
        return 8;
    default:
        ESP_LOGW(TAG, "Unsupported function code: 0x%02X", func);
        return 0;
    }
}