/**
 ******************************************************************************
 * @file    bmp388.h
 * @brief   Public API for the Bosch BMP388 barometric pressure and
 *          temperature sensor.
 * @details
 *   Handle-free, hardware-agnostic driver. All bus traffic goes through
 *   the shared bare-metal SPI1 driver in Protocols/SPI/ (spi_driver.h) —
 *   the BMP388 shares the SPI bus with the ICM20948, so the chip-select
 *   pin is passed on every transaction rather than being owned by this
 *   driver.
 *
 *   Depends only on the standard library and this driver's own headers
 *   (bmp388.h, bmp388_private.h). No HAL_* calls anywhere.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include "bmp388_private.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            PUBLIC DATA TYPES                               *
 *============================================================================*/

/**
 * @brief One compensated BMP388 measurement.
 *
 * @details
 *   Both fields are already in physical SI-ish units — no raw counts
 *   leak out of the driver. pressure is in Pascals (not hPa) so that a
 *   later altitude fusion stage can feed it straight into the
 *   barometric formula without a unit conversion hidden somewhere else.
 */
typedef struct {
    float pressure;      /**< Compensated pressure [Pa]. */
    float temperature;   /**< Compensated die temperature [°C]. */
} BMP388_Data;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Bring the BMP388 up: verify identity, soft-reset, load factory
 *        calibration.
 *
 * @details
 *   Sequence:
 *     1. Read CHIP_ID — must equal BMP388_CHIP_ID_VALUE (0x50).
 *     2. Write SOFT_RESET_CMD to CMD, wait for the part to come back.
 *     3. Burst-read the 21-byte calibration block from NVM and scale it
 *        into the caller-supplied BMP388_CalibData.
 *
 *   The caller MUST keep the calib struct alive for the lifetime of the
 *   driver — BMP388_GetData() needs it on every call. There is no
 *   internal static storage (no hidden state).
 *
 * @param[out] calib  Caller-owned struct that receives the scaled
 *                    calibration coefficients. Must not be NULL.
 *
 * @retval 1  Init succeeded; *calib is populated.
 * @retval 0  CHIP_ID mismatch (wrong part, bad wiring, or CS on the wrong
 *            pin), or calib was NULL.
 */
uint8_t BMP388_Init(BMP388_CalibData *calib);

/**
 * @brief Read the CHIP_ID register (0x00).
 * @return The register value. 0x50 on a genuine BMP388; 0x00 usually
 *         means the SPI bus isn't talking to the part at all.
 *
 * @note  Declared here for debugging / bring-up — BMP388_Init() already
 *        performs the same check internally, so most applications never
 *        need to call this directly.
 */
uint8_t BMP388_ReadChipID(void);

/**
 * @brief Issue a soft reset by writing BMP388_SOFT_RESET_CMD (0xB6) to
 *        the CMD register.
 *
 * @note  Non-blocking: it does not wait for the reset to complete. The
 *        caller is responsible for any delay before the next access.
 *        BMP388_Init() does this internally with a short busy-wait.
 *
 * @warning A soft reset discards the loaded calibration, so
 *          BMP388_ReadCalibData() (or a full BMP388_Init()) must be
 *          called again afterwards.
 */
void BMP388_SoftReset(void);

/**
 * @brief Burst-read the 21-byte factory calibration block from NVM and
 *        scale it into the caller's struct.
 *
 * @param[out] calib  Destination for the scaled coefficients. Must not
 *                    be NULL.
 *
 * @note  Exposed publicly for the same reason as BMP388_ReadChipID() —
 *        a caller who has already reset the part (via BMP388_SoftReset())
 *        can reload calibration without repeating the identity check.
 *        BMP388_Init() calls this internally.
 */
void BMP388_ReadCalibData(BMP388_CalibData *calib);

/**
 * @brief Write the PWR_CTRL register directly.
 *
 * @param[in] value  Raw byte. See bmp388_private.h for the
 *                   BMP388_PWR_CTRL_* macros (SLEEP / FORCED / NORMAL,
 *                   plus PRESS_EN / TEMP_EN bit masks).
 *
 * @note  Low-level escape hatch. Most callers should leave the part in
 *        its power-on defaults and just call BMP388_GetData() — the
 *        sensor is already enabled at reset on most breakouts.
 */
void BMP388_SetPowerCtrl(uint8_t value);

/**
 * @brief Configure pressure and temperature oversampling.
 *
 * @param[in] osr_p  Pressure oversampling: 0..5 (1x, 2x, 4x, 8x, 16x, 32x).
 * @param[in] osr_t  Temperature oversampling: 0..5, same scale.
 *
 * @note  Higher oversampling = less noise but lower max ODR. A good
 *        starting point is (2, 1) — 4x pressure, 2x temperature.
 */
void BMP388_SetOSR(uint8_t osr_p, uint8_t osr_t);

/**
 * @brief Configure the output data rate.
 *
 * @param[in] odr  Raw ODR field: 0..25 (see datasheet Table 11).
 *                 0x00 = 200 Hz, 0x01 = 100 Hz, 0x02 = 50 Hz,
 *                 0x03 = 25 Hz, 0x04 = 12.5 Hz, 0x05 = 6.25 Hz, ...
 *                 Only the low 5 bits are used.
 */
void BMP388_SetODR(uint8_t odr);

/**
 * @brief Read the 24-bit raw pressure and temperature counts in one
 *        contiguous burst (registers 0x04..0x09).
 *
 * @param[out] raw_press  24-bit unsigned pressure ADC value.
 * @param[out] raw_temp   24-bit unsigned temperature ADC value.
 *
 * @note  These are *uncompensated* — see BMP388_GetData() for the
 *        calibrated values, which is what almost every application
 *        actually wants.
 */
void BMP388_ReadRawData(uint32_t *raw_press, uint32_t *raw_temp);

/**
 * @brief Read, compensate, and return one pressure/temperature sample.
 *
 * @details
 *   Combines BMP388_ReadRawData() with the datasheet's compensation
 *   formulas from BST-BMP388-DS001 Section 9.3.1, using the coefficients
 *   previously loaded by BMP388_Init() (or BMP388_ReadCalibData()).
 *
 *   This is the one function a normal application calls in its main loop.
 *
 * @param[in,out] calib  Calibration struct from BMP388_Init(). The
 *                       t_lin field is overwritten on every call — it is
 *                       an intermediate, not a persistent setting.
 * @param[out]    out    Receives the compensated measurement in Pa and
 *                       °C. Must not be NULL.
 */
void BMP388_GetData(BMP388_CalibData *calib, BMP388_Data *out);

#ifdef __cplusplus
}
#endif

#endif /* BMP388_H */
