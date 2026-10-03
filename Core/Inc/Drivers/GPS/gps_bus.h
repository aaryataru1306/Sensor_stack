/**
 * @file    gps_bus.h
 * @brief   Platform bus contract for the GPS driver
 *          (DRIVER_STANDARD.md Section 4, adapted for a streaming UART
 *          device rather than an address-and-register bus).
 * @details gps.c calls only these functions to touch hardware. Exactly
 *          one implementation is linked into any given binary:
 *            - Real hardware:  gps_bus_stm32.c   (bare-metal USART2
 *                               registers via Drivers/UART/uart_bare.c —
 *                               genuinely zero HAL_*, unlike
 *                               bmp388_bus_stm32.c / icm42688_bus_stm32.c,
 *                               which do call HAL_SPI_*; see this
 *                               driver's README "Why this bus file is
 *                               bare-metal and the others aren't")
 *            - QEMU/host test: mock_gps_bus.c   (in-memory scripted NMEA
 *                               byte stream)
 *
 *          Why this contract looks different from bmp388_bus.h /
 *          icm42688_bus.h: I2C/SPI sensors are *addressed* — the driver
 *          picks a register and reads/writes exactly that many bytes. A
 *          GPS module streams unsolicited NMEA text continuously; there
 *          is no register to name. The natural bus primitive here is
 *          "give me the next byte if one is waiting" (non-blocking, one
 *          byte at a time — matching the polling-RXNE bare-metal UART
 *          this task calls for), not "read N bytes starting at address
 *          R." GPS_BusWrite() exists only for the rare case a module or
 *          a UART-controlled OSD (see gps.h's GPS_SendRawCommand())
 *          accepts commands back over the same wire.
 */

#ifndef GPS_BUS_H
#define GPS_BUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Non-blocking single-byte read.
 * @param[in]  bus_context  Opaque context from GPS_Handle_t::bus_context.
 * @param[out] byte_out     Written only when the return is 0.
 * @retval  0  A byte was available and has been written to *byte_out.
 * @retval  1  No byte currently waiting — the normal, expected result on
 *             most calls; callers must not treat this as an error (see
 *             gps.h's GPS_Process()).
 * @retval -1  Hard bus fault (maps to GPS_ERROR_BUS).
 */
int GPS_BusReadByte(void *bus_context, uint8_t *byte_out);

/**
 * @brief Writes length bytes out over the same UART link.
 * @return 0 on success, nonzero on any bus failure.
 */
int GPS_BusWrite(void *bus_context, const uint8_t *data, uint16_t length);

/**
 * @brief Blocking millisecond delay.
 * @param[in] ms Duration to block for.
 */
void GPS_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* GPS_BUS_H */
