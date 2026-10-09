#ifndef GPS_NEOM8N_H
#define GPS_NEOM8N_H

#include <stdint.h>
#include <stdbool.h>
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_NMEA_BUF_SIZE   128U

/**
 * @brief Clean output telemetry structure extracted from NMEA sentences.
 */
typedef struct {
    float latitude;     /**< Decimal degrees (+North, -South) */
    float longitude;    /**< Decimal degrees (+East, -West) */
    float speed_knots;  /**< Ground speed in knots */
    char status;        /**< 'A' = Active/Valid 3D Fix, 'V' = Void/No Fix */
    bool fix_valid;     /**< True when coordinates and fix are valid */
} GPS_Data_t;

/**
 * @brief Internal driver instance and working state buffer.
 * @note  Treat as opaque; do not modify fields directly outside the driver.
 */
typedef struct {
    char rx_buf[GPS_NMEA_BUF_SIZE];
    uint16_t rx_idx;
    UART_Handle_t *huart;
} GPS_Driver_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Binds the GPS driver instance to an initialized UART handle.
 *
 * @param[out] driver  Pointer to the GPS driver instance to initialize.
 * @param[in]  huart   Pointer to an initialized UART handle (e.g. USART2 @ 9600 baud).
 *
 * @note Ensure RCC clocks and GPIO pins for the USART peripheral are configured 
 *       prior to calling this function.
 */
void GPS_Init(GPS_Driver_t *driver, UART_Handle_t *huart);

/**
 * @brief Non-blocking state machine update. Polls UART RX bytes, reconstructs 
 *        NMEA sentences, and updates telemetry when a full sentence arrives.
 *
 * @param[in,out] driver    Pointer to initialized GPS driver instance.
 * @param[out]    out_data  Pointer to telemetry structure receiving fresh data.
 *
 * @return true   A complete, valid NMEA sentence was received and parsed on THIS call.
 * @return false  No new data, sentence incomplete, or corrupt data ignored.
 *
 * @example Usage in main.c:
 * @code
 *   UART_Handle_t huart_gps;
 *   GPS_Driver_t  gps_drv;
 *   GPS_Data_t    gps_data;
 *
 *   // 1. Hardware & UART setup
 *   // UART_Init(&huart_gps, (volatile uint32_t *)0x40004400UL, &uart_cfg);
 *
 *   // 2. Bind GPS driver to hardware handle
 *   GPS_Init(&gps_drv, &huart_gps);
 *
 *   while (1) {
 *       // 3. Poll continuously in main loop (non-blocking)
 *       if (GPS_Update(&gps_drv, &gps_data)) {
 *           // Fresh 3D fix coordinates available
 *           float my_lat = gps_data.latitude;
 *           float my_lon = gps_data.longitude;
 *       }
 *   }
 * @endcode
 */
bool GPS_Update(GPS_Driver_t *driver, GPS_Data_t *out_data);

/**
 * @brief Direct parser helper to extract coordinates from a complete NMEA string.
 *
 * @param[in]  sentence  Null-terminated NMEA string (e.g. "$GPRMC,...").
 * @param[out] out_data  Destination telemetry struct.
 *
 * @return true   Sentence had valid RMC header, integrity star, and 'A' fix status.
 * @return false  Sentence was corrupt, missing fix, or unsupported type.
 */
bool GPS_ParseNMEA(const char *sentence, GPS_Data_t *out_data);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NEOM8N_H */