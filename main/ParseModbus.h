#include "stdio.h"
#include "string.h"

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
int modbus_get_frame_len(uint8_t *buf, int len);