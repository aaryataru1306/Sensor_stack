/**
 ******************************************************************************
 * @file    MPU6050_interface.h
 * @brief   Public API for the InvenSense MPU6050 6-axis IMU
 *          (accelerometer + gyroscope + die temperature).
 * @details
 *   Hardware-agnostic driver. All bus traffic goes through the shared
 *   bare-metal I2C driver in Protocols/I2C/ (i2c_interface.h), and all
 *   timing goes through the TIM protocol layer (TIM_interface.h).
 *
 *   Depends only on the standard library and this driver's own headers
 *   (MPU6050_interface.h, MPU6050_private.h, MPU6050_config.h). No
 *   HAL_* calls anywhere.
 *
 * @note  The MPU6050 has no on-board magnetometer, so — unlike the
 *        MPU9250 — there is no magnetic-field reading and no absolute
 *        heading source.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef MPU6050_INTERFACE_H
#define MPU6050_INTERFACE_H

#include <stdint.h>
#include "ErrTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            PUBLIC DATA TYPES                               *
 *============================================================================*/

/**
 * @brief Processed 6-axis sensor data and temperature.
 *
 * @details  Data is converted from raw bits to physical units
 *           (g, °/s, °C). No raw counts leak out.
 */
typedef struct {
    float AccelX;      /**< Acceleration in X-axis [g]. */
    float AccelY;      /**< Acceleration in Y-axis [g]. */
    float AccelZ;      /**< Acceleration in Z-axis [g]. */
    float GyroX;       /**< Angular velocity in X-axis [°/s]. */
    float GyroY;       /**< Angular velocity in Y-axis [°/s]. */
    float GyroZ;       /**< Angular velocity in Z-axis [°/s]. */
    float Temperature; /**< Chip temperature [°C]. */
} MPU6050_Data_t;

/**
 * @brief 3D spatial coordinates, dead-reckoned from the IMU.
 *
 * @note  X and Y are currently always 0 — the driver only integrates
 *        Z (vertical) from the accelerometer, because it has no
 *        horizontal position reference. A future fusion layer using
 *        heading + speed could populate X and Y.
 */
typedef struct {
    float X;   /**< Current X position [m]. */
    float Y;   /**< Current Y position [m]. */
    float Z;   /**< Current Z position [m]. */
} MPU6050_Position_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Bring the IMU up: reset it, configure it, and average a
 *        still-start sample window into the accelerometer / gyroscope
 *        zero offsets.
 *
 * @details
 *   Also verifies the WHO_AM_I identity register, so a mis-wired or
 *   dead part is reported here rather than showing up later as
 *   plausible-looking garbage.
 *
 * @retval OK             The IMU answered and is configured.
 * @retval NOK            The identity check failed — check the I2C
 *                        wiring (SDA/SCL, pull-ups) and that
 *                        MPU6050_I2C_ADDR matches the level of the
 *                        part's AD0 pin.
 * @retval TIMEOUT_STATE  The part did not respond in time.
 */
ErrorState_t MPU6050_enumInit(void);

/**
 * @brief Read the accelerometer, gyroscope and temperature in one burst.
 *
 * @details
 *   Reading the whole block in a single transfer is what keeps the axes
 *   coherent — separate reads could straddle two samples and describe a
 *   vehicle orientation that never existed.
 *
 * @param[out] Copy_pData  Receives the converted readings, in physical
 *                         units. Must not be NULL.
 *
 * @retval OK            The data was read.
 * @retval NULL_POINTER  Copy_pData was NULL.
 * @retval NOK           The I2C transfer failed.
 */
ErrorState_t MPU6050_enumReadData(MPU6050_Data_t *Copy_pData);

/**
 * @brief Integrated (relative) heading from the Z gyro.
 *
 * @warning The MPU6050 has no magnetometer, so there is nothing to
 *          correct this against: it is a pure integration of GyroZ over
 *          time and, like any dead-reckoned angle, it **drifts without
 *          bound**. It is fine for a short turn (park-assist style
 *          manoeuvres, a few seconds of relative steering feedback);
 *          do not treat it as a compass, and re-zero it (or fuse in an
 *          external reference) for anything that needs to hold an
 *          absolute bearing over minutes.
 *
 * @param[in]  Copy_pData     A fresh reading from MPU6050_enumReadData().
 * @param[in]  Copy_fDt       Time since the previous call [s]. Feeding
 *                            it a wrong value scales the gyro term and
 *                            corrupts the integration.
 * @param[out] Copy_pfHeading Receives the heading [°], 0..360.
 *
 * @retval OK            The heading was updated.
 * @retval NULL_POINTER  Copy_pData or Copy_pfHeading was NULL.
 *
 * @see MPU6050_GYROZ_SIGN
 */
ErrorState_t MPU6050_enumGetHeading(MPU6050_Data_t *Copy_pData,
                                    float Copy_fDt,
                                    float *Copy_pfHeading);

/**
 * @brief Estimate forward speed by integrating acceleration, with
 *        gravity removed.
 *
 * @details
 *   Gravity compensation is not optional: a car on a 5° slope reads a
 *   persistent ~0.09 g along its axis, and integrating that unremoved
 *   invents about 0.9 m/s of speed every second the car sits still.
 *
 * @param[in]  Copy_pData    A fresh reading from MPU6050_enumReadData().
 * @param[in]  Copy_fDt      Time since the previous call [s].
 * @param[out] Copy_pfSpeed  Receives the speed [cm/s], the unit the
 *                           ADAS modules and the DSRC broadcast both
 *                           expect.
 *
 * @retval OK            The speed was updated.
 * @retval NULL_POINTER  Copy_pData or Copy_pfSpeed was NULL.
 *
 * @warning This is dead reckoning from an accelerometer, so the estimate
 *          drifts: any residual bias is integrated without bound. It is
 *          good enough for the short-horizon time-to-collision maths,
 *          and it is not a substitute for a wheel encoder.
 */
ErrorState_t MPU6050_enumGetSpeed(MPU6050_Data_t *Copy_pData,
                                  float Copy_fDt,
                                  float *Copy_pfSpeed);

/**
 * @brief Advance the dead-reckoned position by one step.
 *
 * @details
 *   Projects the current speed along the current heading and accumulates
 *   it, with pitch used to separate real forward motion from the gravity
 *   component.
 *
 * @param[in]  Copy_pData     A fresh reading from MPU6050_enumReadData().
 * @param[in]  Copy_fSpeed    Current speed [cm/s], from
 *                            MPU6050_enumGetSpeed().
 * @param[in]  Copy_fHeading  Current heading [°], from
 *                            MPU6050_enumGetHeading().
 * @param[in]  Copy_fPitch    Current pitch [°], from
 *                            MPU6050_enumGetAttitude().
 * @param[in]  Copy_fDt       Time since the previous call [s].
 * @param[out] Copy_pPos      Receives the updated position. Must not be
 *                            NULL.
 *
 * @retval OK            The position was advanced.
 * @retval NULL_POINTER  Copy_pData or Copy_pPos was NULL.
 *
 * @warning Inherits the drift of MPU6050_enumGetSpeed() and then
 *          integrates it a second time, so the position error grows
 *          quadratically. Treat it as a short-term relative estimate,
 *          never as an absolute location.
 */
ErrorState_t MPU6050_enumGetPosition(MPU6050_Data_t *Copy_pData,
                                     float Copy_fSpeed,
                                     float Copy_fHeading,
                                     float Copy_fPitch,
                                     float Copy_fDt,
                                     MPU6050_Position_t *Copy_pPos);

/**
 * @brief Pitch and roll, from a complementary filter over the
 *        accelerometer and gyro.
 *
 * @details
 *   The accelerometer alone can find "down" but is corrupted by every
 *   bump; the gyro alone is smooth but drifts. Blending them gives an
 *   angle that is both steady and drift-free.
 *
 * @param[in]  Copy_pData    A fresh reading from MPU6050_enumReadData().
 * @param[in]  Copy_fDt      Actual time since the previous call [s].
 *                           The caller passes the measured value rather
 *                           than a nominal one, because a jittery period
 *                           would otherwise mis-scale the gyro term.
 * @param[out] Copy_pfPitch  Receives the pitch [°]. Must not be NULL.
 * @param[out] Copy_pfRoll   Receives the roll [°]. Must not be NULL.
 *
 * @retval OK            Pitch and roll were updated.
 * @retval NULL_POINTER  Any of the pointer arguments was NULL.
 */
ErrorState_t MPU6050_enumGetAttitude(MPU6050_Data_t *Copy_pData,
                                     float Copy_fDt,
                                     float *Copy_pfPitch,
                                     float *Copy_pfRoll);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_INTERFACE_H */
