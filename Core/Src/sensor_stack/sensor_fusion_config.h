/**
 ******************************************************************************
 * @file    sensor_fusion_config.h
 * @brief   Compile-time tuning for the sensor-fusion EKF.
 * @details
 *   Every constant here is a design decision that will need revisiting
 *   against real data. The values below are reasonable starting points
 *   for a ground vehicle on a Black Pill, but the only way to know they
 *   are right is to log the filter's outputs against a known-good
 *   reference (a turntable for attitude, a surveyed position for GPS).
 *
 *   Units follow the EKF convention: gyro and accel noise are given as
 *   *variances* (σ²), not standard deviations, because that's what the
 *   covariance update math actually consumes.
 ******************************************************************************
 */

#ifndef SENSOR_FUSION_CONFIG_H
#define SENSOR_FUSION_CONFIG_H

/*----------------------------------------------------------------------------*
 *                              SAMPLE RATES                                  *
 *----------------------------------------------------------------------------*/

/** @brief Expected IMU sample interval [s]. The filter assumes this is
 *         roughly constant — if you call SF_Update() faster or slower
 *         than this, adjust the Q values accordingly, or better, feed
 *         the filter the actual measured dt. */
#define SF_IMU_DT              0.01f      /* 100 Hz */

/** @brief Gyro measurement noise σ² [rad²/s²]. From the ICM20948
 *         datasheet's noise density (~0.015 dps/√Hz) at 100 Hz
 *         bandwidth, this lands around 1e-5 for a quiet bench. */
#define SF_GYRO_NOISE_VAR      1.0e-5f

/** @brief Gyro bias random-walk σ² [rad²/s³]. How fast the bias is
 *         allowed to wander. Too small → the filter takes forever to
 *         learn the bias. Too large → the filter interprets real motion
 *         as bias. Start here, tighten once you see the estimated bias
 *         settle. */
#define SF_GYRO_BIAS_WALK_VAR  1.0e-9f

/** @brief Accelerometer measurement noise σ² [g²]. Includes both sensor
 *         noise and the fact that during motion the accel *isn't* just
 *         gravity — the filter treats all of that as noise. 1e-2 is
 *         deliberately loose so a bump doesn't yank the attitude. */
#define SF_ACCEL_NOISE_VAR     1.0e-2f

/*----------------------------------------------------------------------------*
 *                          INITIAL UNCERTAINTY                               *
 *----------------------------------------------------------------------------*/

/** @brief Initial attitude uncertainty σ² [quaternion component²]. */
#define SF_INIT_P_ATTITUDE     1.0e-2f

/** @brief Initial gyro-bias uncertainty σ² [(rad/s)²]. */
#define SF_INIT_P_GYRO_BIAS    1.0e-4f

/*----------------------------------------------------------------------------*
 *                          OUTLIER REJECTION                                 *
 *----------------------------------------------------------------------------*/

/** @brief Accelerometer magnitude window [g]. If |a| is outside
 *         [1-margin, 1+margin] the accel reading is rejected for this
 *         update — the vehicle is accelerating hard enough that the
 *         "gravity is 1 g" assumption doesn't hold, and using it would
 *         corrupt attitude. */
#define SF_ACCEL_MAG_MIN       0.75f
#define SF_ACCEL_MAG_MAX       1.25f

/** @brief Mahalanobis distance gate for the accel update. If the
 *         normalized innovation squared exceeds this, the measurement is
 *         treated as an outlier and skipped. χ²(2 dof, 99.9%) ≈ 13.8. */
#define SF_ACCEL_MAHALANOBIS_GATE  13.8f

/*----------------------------------------------------------------------------*
 *                              SANITY CLAMPS                                 *
 *----------------------------------------------------------------------------*/

/** @brief Gyro bias magnitude clamp [rad/s]. Prevents the filter from
 *         winding up to a nonsense bias if it gets a run of corrupted
 *         measurements. ±0.1 rad/s ≈ ±5.7°/s, well above any real bias. */
#define SF_GYRO_BIAS_MAX       0.1f

/** @brief Maximum time [s] between updates before the filter assumes it
 *         lost data and resets the covariance (but keeps the state). */
#define SF_MAX_DT_S            0.5f

#endif /* SENSOR_FUSION_CONFIG_H */
