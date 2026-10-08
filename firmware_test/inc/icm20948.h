/**
 ******************************************************************************
 * @file    icm20948.h
 * @brief   Public API for the TDK InvenSense ICM-20948 9-axis IMU
 *          (accelerometer + gyroscope + temperature exposed; the
 *          AK09916 magnetometer behind the internal I2C master is not
 *          implemented).
 * @details
 *   Hardware-agnostic driver. All bus traffic goes through the shared
 *   bare-metal SPI1 driver in Protocols/SPI/ (spi_driver.h) — the
 *   ICM20948 shares the SPI bus with the BMP388, so the chip-select
 *   pin is passed on every transaction rather than being owned by this
 *   driver.
 *
 *   Depends only on the standard library and this driver's own headers
 *   (icm20948.h, icm20948_private.h). No HAL_* calls anywhere.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            PUBLIC DATA TYPES                               *
 *============================================================================*/

/**
 * @brief Gyroscope full-scale range.
 *
 * @note  A narrower range gives finer resolution but saturates sooner.
 *        For a road vehicle (no spins above a few hundred dps) the
 *        narrowest range is almost always the right trade-off.
 */
typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0x00,
    ICM20948_GYRO_FS_500DPS  = 0x01,
    ICM20948_GYRO_FS_1000DPS = 0x02,
    ICM20948_GYRO_FS_2000DPS = 0x03
} ICM20948_GyroFS_t;

/**
 * @brief Accelerometer full-scale range.
 *
 * @note  Same trade-off as the gyro: ±2 g is the correct default for
 *        ground vehicles (a hard brake is ~1 g; you'd need track use or
 *        crash testing to justify ±4 g or above).
 */
typedef enum {
    ICM20948_ACCEL_FS_2G  = 0x00,
    ICM20948_ACCEL_FS_4G  = 0x01,
    ICM20948_ACCEL_FS_8G  = 0x02,
    ICM20948_ACCEL_FS_16G = 0x03
} ICM20948_AccelFS_t;

#include "icm20948_config.h"

/**
 * @brief Driver status codes.
 */
typedef enum {
    ICM20948_OK = 0,             /**< Operation succeeded. */
    ICM20948_ERROR_WHOAMI = 1,   /**< WHO_AM_I mismatch — wrong part, bad
                                      wiring, or CS on the wrong pin. */
    ICM20948_ERROR_INVALID_PARAM,/**< Null pointer or out-of-range argument. */
    ICM20948_ERROR_NOT_INITIALIZED /**< Called before ICM20948_Init(). */
} ICM20948_Status_t;

/**
 * @brief One 6-axis sample plus die temperature.
 *
 * @details
 *   All fields are already in physical units — no raw counts leak out.
 *   Accel is in g (1 g ≈ 9.80665 m/s²), gyro is in °/s, temp is in °C.
 */
typedef struct {
    float accel_x;   /**< Acceleration X-axis [g]. */
    float accel_y;   /**< Acceleration Y-axis [g]. */
    float accel_z;   /**< Acceleration Z-axis [g]. */
    float gyro_x;    /**< Angular rate X-axis [°/s]. */
    float gyro_y;    /**< Angular rate Y-axis [°/s]. */
    float gyro_z;    /**< Angular rate Z-axis [°/s]. */
    float temp_c;    /**< Die temperature [°C]. */
} ICM20948_Data_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Bring the ICM-20948 up: verify identity, reset, configure
 *        full-scale ranges, wake from sleep.
 *
 * @details
 *   Sequence:
 *     1. Soft-reset via PWR_MGMT_1.DEVICE_RESET, wait for the part to
 *        settle (bus is silent long enough for the reset to complete).
 *     2. Read WHO_AM_I — must equal 0xEA.
 *     3. Wake up: clear SLEEP, select auto clock source (PLL when
 *        available, internal oscillator fallback).
 *     4. Enable accel + gyro (clear DISABLE_ACCEL / DISABLE_GYRO in
 *        PWR_MGMT_2).
 *     5. Configure gyro and accel full-scale ranges in Bank 2.
 *     6. Return the bank selector to Bank 0 for normal operation.
 *
 *   The caller must have already brought up SPI1 (SPI1_Init()) before
 *   calling this — the SPI peripheral is shared with the BMP388 and is
 *   therefore owned by main.c, not by this driver.
 *
 * @param[in] accel_fs  Accelerometer full-scale range.
 * @param[in] gyro_fs   Gyroscope full-scale range.
 *
 * @retval ICM20948_OK                 Init succeeded; part is configured.
 * @retval ICM20948_ERROR_WHOAMI       WHO_AM_I did not read 0xEA.
 */
ICM20948_Status_t ICM20948_Init(ICM20948_AccelFS_t accel_fs,
                                ICM20948_GyroFS_t  gyro_fs);

/**
 * @brief Read the WHO_AM_I register (Bank 0, 0x00).
 *
 * @return The register value. 0xEA on a genuine ICM-20948; 0x00 usually
 *         means the SPI bus isn't talking to the part at all.
 *
 * @note  Exposed for debugging / bring-up — ICM20948_Init() already
 *        performs the same check internally, so most applications never
 *        need to call this directly.
 */
uint8_t ICM20948_WhoAmI(void);

/**
 * @brief Read accelerometer, gyroscope and temperature in one burst and
 *        convert to physical units.
 *
 * @details
 *   Reading the whole block in a single SPI transaction is what keeps
 *   the axes coherent — separate reads could straddle two samples and
 *   describe an orientation that never existed.
 *
 * @param[out] data  Destination for the converted sample. Must not be
 *                   NULL.
 */
void ICM20948_ReadAll(ICM20948_Data_t *data);

/**
 * @brief Read just the accelerometer (converted to g).
 *
 * @param[out] ax  X-axis acceleration [g].
 * @param[out] ay  Y-axis acceleration [g].
 * @param[out] az  Z-axis acceleration [g].
 *
 * @note  Prefer ICM20948_ReadAll() unless you specifically need a
 *        narrower read — one burst is cheaper than three separate
 *        transactions and avoids the axis-coherency hazard.
 */
void ICM20948_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief Read just the gyroscope (converted to °/s).
 *
 * @param[out] gx  X-axis angular rate [°/s].
 * @param[out] gy  Y-axis angular rate [°/s].
 * @param[out] gz  Z-axis angular rate [°/s].
 *
 * @note  Same preference note as ICM20948_ReadAccel().
 */
void ICM20948_ReadGyro(float *gx, float *gy, float *gz);

/**
 * @brief Read just the die temperature (converted to °C).
 *
 * @param[out] temp_c  Die temperature [°C].
 */
void ICM20948_ReadTemp(float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
