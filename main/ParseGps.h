#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#define GPS_BUF_SIZE 256
bool gps_validate_checksum(const char *nmea_sentence);
typedef struct
{
    float latitude;
    float longitude;
    float speed_knots;
    float course;
    char status;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    int fix_quality;
    int satellites;
    float altitude;
    bool valid;
    TickType_t last_update_tick; /**< xTaskGetTickCount() when last valid RMC received */
} gps_data_t;
extern gps_data_t gps;
void gps_process_line(const char *line);