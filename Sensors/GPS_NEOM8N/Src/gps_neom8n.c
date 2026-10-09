#include "gps_neom8n.h"
#include <stdlib.h>
#include <string.h>

void GPS_Init(GPS_Driver_t *driver, UART_Handle_t *huart) {
    if (driver == NULL || huart == NULL) {
        return;
    }
    driver->huart = huart;
    driver->rx_idx = 0U;
    memset(driver->rx_buf, 0, sizeof(driver->rx_buf));
}

/**
 * @brief Helper to find comma-delimited fields without modifying the string
 */
static const char* NMEA_GetField(const char *str, uint8_t field_num) {
    uint8_t count = 0U;
    while (*str && *str != '\r' && *str != '\n' && *str != '*') {
        if (count == field_num) {
            return str;
        }
        if (*str == ',') {
            count++;
        }
        str++;
    }
    return NULL;
}

bool GPS_ParseNMEA(const char *sentence, GPS_Data_t *out_data) {
    if (sentence == NULL || out_data == NULL) {
        return false;
    }

    /* 1. Filter for RMC sentences ($GPRMC or $GNRMC) */
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return false;
    }

    /* 2. Check for sentence integrity delimiter */
    if (strchr(sentence, '*') == NULL) {
        out_data->fix_valid = false;
        return false;
    }

    /* 3. Extract Status (Field 2) */
    const char *pStatus = NMEA_GetField(sentence, 2U);
    if (pStatus == NULL) {
        out_data->fix_valid = false;
        return false;
    }

    out_data->status = *pStatus;
    if (out_data->status != 'A') {
        out_data->fix_valid = false;
        return false;
    }

    /* 4. Extract Lat (Field 3), N/S (Field 4), Lon (Field 5), E/W (Field 6) */
    const char *pLat = NMEA_GetField(sentence, 3U);
    const char *pNS  = NMEA_GetField(sentence, 4U);
    const char *pLon = NMEA_GetField(sentence, 5U);
    const char *pEW  = NMEA_GetField(sentence, 6U);
    const char *pSpd = NMEA_GetField(sentence, 7U);

    if (!pLat || !pNS || !pLon || !pEW) {
        out_data->fix_valid = false;
        return false;
    }

    /* Convert Latitude (DDMM.MMMM -> Decimal Degrees) */
    float raw_lat = (float)atof(pLat);
    int lat_deg = (int)(raw_lat / 100.0f);
    float lat_min = raw_lat - ((float)lat_deg * 100.0f);
    out_data->latitude = (float)lat_deg + (lat_min / 60.0f);
    if (*pNS == 'S' || *pNS == 's') {
        out_data->latitude = -out_data->latitude;
    }

    /* Convert Longitude (DDDMM.MMMM -> Decimal Degrees) */
    float raw_lon = (float)atof(pLon);
    int lon_deg = (int)(raw_lon / 100.0f);
    float lon_min = raw_lon - ((float)lon_deg * 100.0f);
    out_data->longitude = (float)lon_deg + (lon_min / 60.0f);
    if (*pEW == 'W' || *pEW == 'w') {
        out_data->longitude = -out_data->longitude;
    }

    /* Speed in Knots */
    if (pSpd != NULL) {
        out_data->speed_knots = (float)atof(pSpd);
    } else {
        out_data->speed_knots = 0.0f;
    }

    out_data->fix_valid = true;
    return true;
}

/**
 * @brief Non-blocking poll: reads incoming bytes from Praprara's UART driver
 *        and parses complete lines automatically.
 */
bool GPS_Update(GPS_Driver_t *driver, GPS_Data_t *out_data) {
    if (driver == NULL || driver->huart == NULL || out_data == NULL) {
        return false;
    }

    uint8_t byte = 0U;
    bool sentence_parsed = false;

    /* Non-blocking poll loop over UART RX */
    while (UART_ReceiveByte(driver->huart, &byte) == UART_OK) {
        /* Start of NMEA sentence */
        if (byte == '$') {
            driver->rx_idx = 0U;
        }

        if (driver->rx_idx < (GPS_NMEA_BUF_SIZE - 1U)) {
            driver->rx_buf[driver->rx_idx++] = (char)byte;
        }

        /* End of line reached */
        if (byte == '\n') {
            driver->rx_buf[driver->rx_idx] = '\0';
            sentence_parsed = GPS_ParseNMEA(driver->rx_buf, out_data);
            driver->rx_idx = 0U;
            
            if (sentence_parsed) {
                return true;
            }
        }
    }

    return false;
}