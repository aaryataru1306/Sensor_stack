/**
 ******************************************************************************
 * @file    sensor_task.h
 * @brief   Application-layer sensor task: owns initialization, calibration,
 *          cross-verification, fusion, drift monitoring, and the single
 *          SensorSnapshot_t that downstream consumers (FSM, control logic,
 *          telemetry) read.
 * @details
 *   Layering:
 *
 *     Protocols (SPI/I2C/UART)  ←  Drivers (ICM/MPU/BMP/GPS)
 *                                            ↓
 *                                       sensor_stack (EKF + alt filter)
 *                                            ↓
 *                                       sensor_task  ← THIS LAYER
 *                                            ↓
 *                                    FSM / control / telemetry
 *
 *   The whole point of this file is that the layer below publishes raw
 *   sensor values and fused estimates in *its* units and *its* structs,
 *   and the layer above wants "the current state of the drone" in one
 *   call. SensorTask_GetSnapshot() is that call.
 *
 *   Everything here is synchronous and non-blocking. Call
 *   SensorTask_Update() from your main loop (or from a fixed-rate timer
 *   ISR, ideally) at the loop rate configured in sensor_task_config.h.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              STATUS CODES                                  *
 *============================================================================*/

typedef enum {
  SENSOR_TASK_OK = 0,                /**< Update / call succeeded. */
  SENSOR_TASK_ERROR_INIT_FAILED,     /**< One of the sensor inits failed. */
  SENSOR_TASK_ERROR_NOT_INITIALIZED, /**< Called before SensorTask_Init(). */
  SENSOR_TASK_ERROR_NOT_CALIBRATED,  /**< Called before calibration
                                          completed. */
  SENSOR_TASK_ERROR_FUSION_DIVERGED  /**< EKF covariance lost positive-
                                          definiteness; caller should
                                          expect transiently poor values
                                          and may choose to re-init the
                                          filter via SensorTask_Recalibrate(). */
} SensorTask_Status_t;

/*============================================================================*
 *                              HEALTH FLAGS                                  *
 *============================================================================*/

/**
 * @brief Per-sensor health, updated on every SensorTask_Update().
 *
 * @details
 *   These are *latching* within a health window — a single glitch does
 *   not immediately mark a sensor dead, but a sustained failure (see
 *   SENSOR_TASK_HEALTH_WINDOW in sensor_task_config.h) does. That
 *   hysteresis is what stops a momentary SPI hiccup from tripping the
 *   FSM into a failsafe.
 */
typedef enum {
  SENSOR_HEALTH_UNKNOWN = 0, /**< Not yet read / not yet known. */
  SENSOR_HEALTH_OK,          /**< Reading plausible, agreeing with peers. */
  SENSOR_HEALTH_SUSPECT,     /**< One or two bad readings in the window. */
  SENSOR_HEALTH_FAILED,      /**< Sustained bad readings — do not trust. */
  SENSOR_HEALTH_DISAGREES    /**< Readings consistently disagree with a
                                  peer sensor (cross-verification
                                  failure). */
} SensorHealth_t;

/**
 * @brief Per-sensor status block, exposed in SensorSnapshot_t.
 */
typedef struct {
  SensorHealth_t icm_health;
  SensorHealth_t mpu_health;
  SensorHealth_t bmp_health;
  SensorHealth_t gps_health;

  uint32_t icm_bad_readings; /**< Rolling counters for telemetry. */
  uint32_t mpu_bad_readings;
  uint32_t bmp_bad_readings;
  uint32_t gps_bad_readings;

  uint32_t icm_disagree_count; /**< Times ICM disagreed with MPU. */
  uint32_t mpu_disagree_count;

  uint32_t fusion_diverge_count; /**< Times the EKF covariance went bad. */
} SensorHealthBlock_t;

/*============================================================================*
 *                              CALIBRATION STATE                             *
 *============================================================================*/

typedef enum {
  SENSOR_CALIB_IDLE = 0,    /**< Not started. */
  SENSOR_CALIB_IN_PROGRESS, /**< Averaging samples. */
  SENSOR_CALIB_COMPLETE,    /**< Done successfully. */
  SENSOR_CALIB_FAILED       /**< Not enough still samples, or timeout. */
} SensorCalibState_t;

/*============================================================================*
 *                              SENSOR SNAPSHOT                               *
 *============================================================================*/

/**
 * @brief The one struct downstream consumers read.
 *
 * @details
 *   Everything the FSM / control logic needs, already filtered and
 *   unit-converted, in a single self-contained block. Downstream code
 *   never touches an ICM20948_Data_t or an SF_Attitude_t.
 *
 *   Units:
 *     - attitude angles: degrees
 *     - angular rates:   degrees per second (body frame)
 *     - accel:           g (body frame)
 *     - altitude:        metres above the calibration ground reference
 *     - vertical speed:  metres per second
 *     - lat/lon:         degrees, WGS-84
 *     - ground speed:    metres per second
 *     - course:          degrees, 0..360
 *     - pressure:        Pascals
 *     - temperature:     degrees Celsius
 *
 * @note  This struct is expected to grow. Adding a field here is cheap;
 *        adding a *dependency* from downstream onto a specific driver
 *        is not. New fields should always come through this path.
 */
typedef struct {
  /* --- Timestamp and validity --- */
  uint32_t tick_ms;      /**< Millisecond tick at which this
                              snapshot was produced. */
  uint32_t update_count; /**< Monotonic update counter. */
  bool valid;            /**< True once calibration is complete
                              AND at least one fused update has
                              run. Downstream must check this. */

  /* --- Fused attitude (EKF output) --- */
  float qw, qx, qy, qz; /**< Attitude quaternion (body → world). */
  float roll_deg;       /**< [-180, +180] */
  float pitch_deg;      /**< [-90,  +90]  */
  float yaw_deg;        /**< [0, 360) — relative, no magnetometer */

  /* --- Body-frame angular rates (from primary gyro) --- */
  float gyro_x_dps; /**< Bias-corrected gyro X [°/s]. */
  float gyro_y_dps;
  float gyro_z_dps;

  /* --- Body-frame linear acceleration (gravity-compensated) --- */
  float accel_x_g; /**< Linear accel X [g], gravity removed. */
  float accel_y_g;
  float accel_z_g;

  /* --- Estimated gyro bias (for telemetry / diagnostics) --- */
  float gyro_bias_x_rps;
  float gyro_bias_y_rps;
  float gyro_bias_z_rps;

  /* --- Altitude (fused baro + GPS) --- */
  float altitude_m;         /**< Fused altitude above ground ref. */
  float vertical_speed_m_s; /**< Fused vertical speed. */

  /* --- Barometric sensor (raw, for reference / OSD) --- */
  float pressure_pa;
  float temperature_c;
  float baro_altitude_m; /**< BMP388-only altitude, uncorrected. */

  /* --- GPS (position + ground velocity) --- */
  bool gps_fix_valid;      /**< True if the fix passed validity checks. */
  uint8_t gps_fix_quality; /**< GPS_FixQuality_t value. */
  uint8_t gps_satellites;  /**< Satellites in use. */
  double gps_latitude_deg;
  double gps_longitude_deg;
  float gps_altitude_m;
  float gps_speed_m_s;  /**< Ground speed. */
  float gps_course_deg; /**< Course over ground. */

  /* --- Health / calibration --- */
  SensorCalibState_t calib_state;
  SensorHealthBlock_t health;

  /* --- Convenience flags for the FSM --- */
  bool imu_trustworthy;      /**< At least one IMU is OK *and* the
                                  fused attitude is stable. */
  bool altitude_trustworthy; /**< Baro is OK and no divergence. */
  bool position_trustworthy; /**< GPS has a valid 3D fix. */
} SensorSnapshot_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Bring up every sensor, the fusion filters, and the calibration
 *        state machine.
 *
 * @details
 *   Runs once at boot, after the protocol layers are up (SPI1, I2C1,
 *   USART2 clocks + pins must already be configured — this function
 *   does NOT do that, because those are board bring-up, not sensor
 *   bring-up; see main.c).
 *
 *   The function:
 *     1. Calls each sensor driver's Init() in the right order
 *        (ICM → BMP388 → MPU6050 → GPS).
 *     2. Runs IMU still-start calibration — reads N samples from both
 *        IMUs while the board is expected to be stationary and averages
 *        them into the bias estimates (the ICM's and MPU's own internal
 *        offsets; the fusion layer's gyro-bias state is separate).
 *     3. Captures a barometric ground reference (mean of M samples) so
 *        altitude reads 0 m at the point of calibration.
 *     4. Seeds the fusion EKF from the ICM accel.
 *     5. Marks the calibration state COMPLETE (or FAILED).
 *
 *   The board **must be stationary** during this call. If you need to
 *   defer calibration until after motor arming or similar, call
 *   SensorTask_Init() with the sensors up but skip calibration, then
 *   call SensorTask_Recalibrate() when the board is finally still.
 *
 * @retval SENSOR_TASK_OK on full success (all sensors up, calibration
 *         completed).
 * @retval SENSOR_TASK_ERROR_INIT_FAILED if any sensor did not respond.
 *         The failing sensor will still be marked FAILED in the health
 *         block and the task will run in degraded mode — the caller
 *         decides whether to abort the mission.
 */
SensorTask_Status_t SensorTask_Init(void);

/**
 * @brief Run the periodic task. Call this once per main-loop tick.
 *
 * @details
 *   Does, in order:
 *     1. Reads both IMUs.
 *     2. Cross-verifies their accelerometer magnitudes. If they disagree
 *        by more than the configured threshold, both are marked
 *        SUSPECT for this tick and neither is fed to the EKF (the
 *        filter's own gyro prediction carries it through the tick).
 *        If a *sustained* disagreement is detected, both are marked
 *        DISAGREES and the snapshot's imu_trustworthy goes false.
 *     3. Runs the attitude EKF with the trusted IMU sample.
 *     4. Reads the BMP388 and feeds the altitude filter.
 *     5. Reads the GPS (non-blocking) and, on a valid fix, feeds its
 *        altitude into the altitude filter.
 *     6. Runs the drift monitor — every DriftMonitorPeriod, checks
 *        whether the attitude/altitude have moved more than the
 *        configured drift threshold while the vehicle was supposed to
 *        be stationary. Updates health accordingly.
 *     7. Publishes a fresh SensorSnapshot_t.
 *
 *   Runs in well under 1 ms at 100 MHz, so it is safe to call from a
 *   timer ISR if you want deterministic timing.
 *
 * @retval SENSOR_TASK_OK on a normal tick.
 * @retval SENSOR_TASK_ERROR_NOT_INITIALIZED if Init() has not run.
 * @retval SENSOR_TASK_ERROR_FUSION_DIVERGED if the EKF covariance went
 *         bad this tick — the snapshot is still published, but
 *         imu_trustworthy is forced false. Caller should probably
 *         trigger SensorTask_Recalibrate() when convenient.
 */
SensorTask_Status_t SensorTask_Update(void);

/**
 * @brief Get the current snapshot.
 *
 * @details
 *   Cheap — just a struct copy. Downstream consumers call this every
 *   tick. Never call it from an ISR *at the same priority* as
 *   SensorTask_Update(); use a lower priority, or copy the snapshot
 *   into a buffer from the same context that calls Update().
 *
 * @param[out] out  Filled in with the latest snapshot. Must not be NULL.
 *
 * @retval SENSOR_TASK_OK                   Snapshot populated.
 * @retval SENSOR_TASK_ERROR_NOT_INITIALIZED Called before Init().
 * @retval SENSOR_TASK_ERROR_INVALID_PARAM   out was NULL.
 */
SensorTask_Status_t SensorTask_GetSnapshot(SensorSnapshot_t *out);

/**
 * @brief Re-run calibration and reset the fusion filters.
 *
 * @details
 *   Intended call sites:
 *     - On the ground after a "the IMU is drifting" fault is detected
 *       (check snapshot.health and snapshot.imu_trustworthy first).
 *     - After a power-on delay while the board settles.
 *     - Before a mission, if the previous calibration is old and the
 *       vehicle has been moved or its temperature has changed
 *       significantly.
 *
 *   The board **must be stationary** when this is called. This blocks
 *   for the duration of the sample window (see
 *   SENSOR_TASK_CALIB_SAMPLES and SENSOR_TASK_CALIB_SAMPLE_DELAY_MS in
 *   sensor_task_config.h), so do not call it from a time-critical
 *   context.
 *
 * @retval SENSOR_TASK_OK on success.
 * @retval SENSOR_TASK_ERROR_NOT_INITIALIZED if Init() has not run.
 * @retval SENSOR_TASK_ERROR_INIT_FAILED if any sensor failed during the
 *         calibration sample collection.
 */
SensorTask_Status_t SensorTask_Recalibrate(void);

/**
 * @brief Signal to the drift monitor that the vehicle is now expected
 *        to be stationary (armed but on the ground, or disarmed).
 *
 * @details
 *   The drift monitor can only distinguish "IMU is drifting" from "the
 *   vehicle is actually moving" if it knows which is expected. The
 *   caller tells it via this function. Defaults to "stationary" at
 *   boot, so a bare Init() + Update() loop works without this call.
 *
 * @param[in] stationary  true = the drift monitor will now flag any
 *                        sustained motion as suspected drift; false =
 *                        drift monitoring is suspended (the vehicle is
 *                        flying and motion is expected).
 */
void SensorTask_SetStationary(bool stationary);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TASK_H */