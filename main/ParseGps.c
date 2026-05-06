#include "ParseGps.h"
#include "esp_system.h"
#include "esp_log.h"

gps_data_t gps;
typedef void (*nmea_handler_t)(const char *sentence, char **tok, int count);

typedef struct
{
    const char *prefix;
    nmea_handler_t handler;
} nmea_dispatch_t;
static void handle_rmc(const char *s, char **tok, int count);
static void handle_gga(const char *s, char **tok, int count);
static void handle_gsa(const char *s, char **tok, int count);
static void handle_gsv(const char *s, char **tok, int count);
static void handle_vtg(const char *s, char **tok, int count);
static void handle_zda(const char *s, char **tok, int count);
static void handle_gst(const char *s, char **tok, int count);
static const nmea_dispatch_t nmea_table[] = {
    {"RMC", handle_rmc}, // oke
    {"GGA", handle_gga}, // oke
    {"GSA", handle_gsa}, // oke
    {"GSV", handle_gsv}, // oke
    {"VTG", handle_vtg}, // oke
    {"ZDA", handle_zda}, // oke
    {"GST", handle_gst}, // oke
};
#define NMEA_TABLE_SIZE (sizeof(nmea_table) / sizeof(nmea_dispatch_t))
static int nmea_tokenize(char *buf, char **tok, int max)
{
    int count = 0;
    char *p = buf;

    while (p && count < max)
    {
        tok[count++] = p;

        /* Find next delimiter: ',' or '*' */
        char *end = p;
        while (*end && *end != ',' && *end != '*')
            end++;

        if (*end == '\0')
            break; /* end of string */

        *end = '\0'; /* null-terminate current token */
        p = end + 1; /* next token starts right after delimiter */
    }

    return count;
}

bool gps_validate_checksum(const char *nmea_sentence)
{
    // Calculate checksum by XORing all characters between '$' and '*'
    uint8_t checksum = 0;
    const char *ptr = nmea_sentence + 1; // Skip '$' character

    while (*ptr && *ptr != '*')
    {
        checksum ^= (uint8_t)(*ptr);
        ptr++;
    }

    if (*ptr == '*')
    {
        uint8_t received_checksum = (uint8_t)strtol(ptr + 1, NULL, 16);
        return checksum == received_checksum;
    }
    return false; // Did not find '*' or format error
}
void gps_process_line(const char *line)
{
    if (!gps_validate_checksum(line))
    {
        ESP_LOGW("GPS", "Checksum fail: %.6s", line);
        return;
    }

    // get 3-character message type: $GNRMC → "RMC"
    if (strlen(line) < 6)
        return;
    const char *msg_type = line + 3; // skip $GN / $GP / $GL / $GA / $GB

    char buf[GPS_BUF_SIZE];
    strncpy(buf, line, sizeof(buf) - 1);

    char *tok[30];
    int count = nmea_tokenize(buf, tok, 30);

    // find handler
    bool found = false;
    for (int i = 0; i < NMEA_TABLE_SIZE; i++)
    {
        if (strncmp(msg_type, nmea_table[i].prefix, 3) == 0)
        {
            nmea_table[i].handler(line, tok, count);
            found = true;
            break;
        }
    }

    if (!found)
        ESP_LOGD("GPS", "No handler: %.6s", line);
}
static void handle_rmc(const char *s, char **tok, int count)
{
    if (count < 10)
        return;
    // tok[0] = $GNRMC
    // tok[1] = time, tok[2] = status ...
    gps.status = tok[2][0];
    gps.valid = (gps.status == 'A');
    if (!gps.valid)
        return;

    // time
    gps.hour = (tok[1][0] - '0') * 10 + (tok[1][1] - '0');
    gps.minute = (tok[1][2] - '0') * 10 + (tok[1][3] - '0');
    gps.second = (tok[1][4] - '0') * 10 + (tok[1][5] - '0');

    // lat/lon
    float lat_raw = atof(tok[3]);
    int lat_deg = (int)(lat_raw / 100);
    gps.latitude = lat_deg + (lat_raw - lat_deg * 100) / 60.0f;
    if (tok[4][0] == 'S')
        gps.latitude = -gps.latitude;

    float lon_raw = atof(tok[5]);
    int lon_deg = (int)(lon_raw / 100);
    gps.longitude = lon_deg + (lon_raw - lon_deg * 100) / 60.0f;
    if (tok[6][0] == 'W')
        gps.longitude = -gps.longitude;

    gps.speed_knots = atof(tok[7]);
    gps.course = atof(tok[8]);

    // date
    gps.day = (tok[9][0] - '0') * 10 + (tok[9][1] - '0');
    gps.month = (tok[9][2] - '0') * 10 + (tok[9][3] - '0');
    gps.year = 2000 + (tok[9][4] - '0') * 10 + (tok[9][5] - '0');

    ESP_LOGI("GPS", "RMC: %.6f %s, %.6f %s, %.2f knots",
             gps.latitude, tok[4],
             gps.longitude, tok[6],
             gps.speed_knots);
}

static void handle_gga(const char *s, char **tok, int count)
{
    if (count < 10)
        return;
    // tok[6] = fix quality, tok[7] = satellites, tok[9] = altitude
    gps.fix_quality = atoi(tok[6]);
    gps.satellites = atoi(tok[7]);
    gps.altitude = atof(tok[9]);

    ESP_LOGI("GPS", "GGA: alt=%.1fm sats=%d fix=%d",
             gps.altitude, gps.satellites, gps.fix_quality);
}

static void handle_gsa(const char *s, char **tok, int count)
{
    // tok[2] = fix type (1=none, 2=2D, 3=3D)
    // tok[15] = PDOP, tok[16] = HDOP, tok[17] = VDOP
    ESP_LOGI("GPS", "GSA: fix=%s PDOP=%s HDOP=%s VDOP=%s",
             tok[2], tok[15], tok[16], tok[17]);
}

static void handle_gsv(const char *s, char **tok, int count)
{
    // tok[1]=total msgs, tok[2]=msg num, tok[3]=total sats
    ESP_LOGI("GPS", "GSV: %s sats in view", tok[3]);
}

static void handle_vtg(const char *s, char **tok, int count)
{
    // tok[1]=course true, tok[5]=speed knots, tok[7]=speed km/h
    float speed_kmh = atof(tok[7]);
    ESP_LOGI("GPS", "VTG: %.2f km/h", speed_kmh);
}

static void handle_zda(const char *s, char **tok, int count)
{
    // tok[1]=time, tok[2]=day, tok[3]=month, tok[4]=year
    ESP_LOGI("GPS", "ZDA: %s/%s/%s %s", tok[2], tok[3], tok[4], tok[1]);
}

static void handle_gst(const char *s, char **tok, int count)
{
    // error statistics - usually skipped
    ESP_LOGI("GPS", "GST: lat_err=%s lon_err=%s", tok[6], tok[7]);
}