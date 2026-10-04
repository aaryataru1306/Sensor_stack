/**
 ******************************************************************************
 * @file    sensor_task_config.h
 * @brief   Compile-time tuning for the sensor task: sample windows,
 *          cross-verification thresholds, and drift-monitor settings.
 ******************************************************************************
 */

#ifndef SENSOR_TASK_CONFIG_H
#define SENSOR_TASK_CONFIG_H

/*----------------------------------------------------------------------------*
 *                              NOMINAL RATES                                 *
 *----------------------------------------------------------------------------*/

/** @brief Nominal interval between SensorTask_Update() calls [s]. Used
 *         as dt for the EKF. Replace with a real measured dt once you
 *         have a timer; the fusion correctness is not affected, only
 *         the convergence rate. */
#define SENSOR_TASK_DT_S 0.01f

/** @brief Nominal IMU update rate [Hz], derived. */
#define SENSOR_TASK_IMU_HZ (1.0f / SENSOR_TASK_DT_S)

/*----------------------------------------------------------------------------*
 *                          CALIBRATION WINDOWS                               *
 *----------------------------------------------------------------------------*/

/** @brief Number of samples averaged for the IMU still-start calibration.
 *         400 samples at 2 ms each ≈ 0.8 s. */
#define SENSOR_TASK_CALIB_SAMPLES 400U

/** @brief Delay between calibration samples [ms]. */
#define SENSOR_TASK_CALIB_SAMPLE_DELAY_MS 2U

/** @brief Number of BMP388 samples averaged to establish the ground
 *         altitude reference. */
#define SENSOR_TASK_BARO_REF_SAMPLES 50U

/** @brief If calibration fails to see the IMU produce a plausible
 *         stationary sample within this many attempts, mark FAILED. */
#define SENSOR_TASK_CALIB_MAX_ATTEMPTS (SENSOR_TASK_CALIB_SAMPLES * 2U)

/*----------------------------------------------------------------------------*
 *                          CROSS-VERIFICATION                                *
 *----------------------------------------------------------------------------*/

/** @brief Maximum allowed |‖a_ICM‖ − ‖a_MPU‖| [g] before the two IMUs are
 *         considered to disagree on this tick. Both parts are 3-axis
 *         ±2 g parts at ±0.02 g noise, so 0.15 g is a comfortable margin
 *         above the noise floor. */
#define SENSOR_TASK_IMU_MAG_DISAGREE_G 0.15f

/** @brief Maximum allowed component-wise accel disagreement [g]. Catches
 *         the case where the two magnitudes happen to be similar but
 *         the axes disagree (e.g. one IMU mounted rotated). */
#define SENSOR_TASK_IMU_AXIS_DISAGREE_G 0.25f

/** @brief Rolling window length for health latching [ticks]. */
#define SENSOR_TASK_HEALTH_WINDOW 50U

/** @brief A sensor whose bad-reading count in the window reaches this
 *         is marked SUSPECT; at 2× this value it is marked FAILED. */
#define SENSOR_TASK_HEALTH_SUSPECT_TH 5U

/*----------------------------------------------------------------------------*
 *                          DRIFT MONITOR                                     *
 *----------------------------------------------------------------------------*/

/** @brief How often to run the drift check [ticks]. At 100 Hz this is
 *         once every 5 s, which is enough to catch the slow bias-like
 *         drifts we care about without being fooled by noise. */
#define SENSOR_TASK_DRIFT_CHECK_INTERVAL 500U

/** @brief Warm-up period after calibration during which drift is not
 *         checked [ticks]. The EKF needs a few seconds to converge the
 *         gyro-bias estimate; checking before it has converged would
 *         false-positive every time. */
#define SENSOR_TASK_DRIFT_WARMUP_TICKS 300U

/** @brief Drift threshold on roll/pitch [deg]. If the vehicle is
 *         supposed to be stationary and roll or pitch has changed by
 *         more than this since the last check, flag suspected drift. */
#define SENSOR_TASK_DRIFT_ATTITUDE_DEG 1.5f

/** @brief Drift threshold on altitude [m]. Same idea for the baro —
 *         a real stationary vehicle should not see its altitude estimate
 *         wander by more than this over a drift-check interval. */
#define SENSOR_TASK_DRIFT_ALTITUDE_M 0.75f

/** @brief Number of consecutive drift checks that must trip before the
 *         drift-monitor marks the IMU SUSPECT. One noisy check is not
 *         a drift; three in a row is. */
#define SENSOR_TASK_DRIFT_TRIP_COUNT 3U

/*----------------------------------------------------------------------------*
 *                              GPS THRESHOLDS                                *
 *----------------------------------------------------------------------------*/

/** @brief Minimum satellites for a GPS fix to be considered usable for
 *         altitude fusion. */
#define SENSOR_TASK_GPS_MIN_SATS 4U

/** @brief Minimum fix quality (GPS_FixQuality_t) for a GPS fix to be
 *         considered usable for altitude fusion. */
#define SENSOR_TASK_GPS_MIN_FIX_QUALITY 1U /* GPS_FIX_QUALITY_GPS */

#endif /* SENSOR_TASK_CONFIG_H */