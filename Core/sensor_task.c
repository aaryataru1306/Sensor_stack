/**
 ******************************************************************************
 * @file    sensor_task.c
 * @brief   Implementation of the application-layer sensor task declared
 *          in sensor_task.h.
 * @details
 *   See sensor_task.h for the layering rationale. This file is the only
 *   place in the tree that reads *all* sensors, so it is also the only
 *   place that can do cross-verification between them.
 ******************************************************************************
 */

#include "sensor_task.h"
#include "sensor_task_config.h"

#include "i2c_interface.h"
#include "spi_driver.h"
#include "uart_bare.h"


#include "MPU6050_interface.h"
#include "bmp388.h"
#include "icm20948.h"


#include "gps.h"
#include "gps_nmea.h"
#include "gps_ringbuffer.h"

#include "sensor_fusion.h"

#include <math.h>
#include <string.h>


/*============================================================================*
 *                          INTERNAL STATE                                    *
 *============================================================================*/

/* Raw sensor readings and driver storage — private to this file. */
static ICM20948_Data_t s_icm;
static BMP388_CalibData s_bmp_calib;
static BMP388_Data s_bmp;
static MPU6050_Data_t s_mpu;

static GPS_RingBuffer_t s_gps_ring;
static NMEA_Parser_t s_gps_parser;
static GPS_Handle_t s_gps;
static GPS_Data_t s_gps_fix;

/* Fusion output. */
static SF_Attitude_t s_attitude;
static SF_Altitude_t s_altitude;

/* Current published snapshot. */
static SensorSnapshot_t s_snapshot;

/* Calibration state. */
static SensorCalibState_t s_calib_state = SENSOR_CALIB_IDLE;
static bool s_initialized = false;
static bool s_stationary = true; /* default: assume still */

/* Gyro/accel zero offsets from the still-start calibration (per IMU).
 * These are the IMU's own residual offsets, distinct from the fusion
 * layer's gyro-bias state. */
static float s_icm_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_mpu_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_icm_accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_mpu_accel_bias[3] = {0.0f, 0.0f, 0.0f};

/* Barometric ground reference. */
static float s_ground_pressure_pa = 0.0f;

/* Drift monitor state. */
static uint32_t s_drift_tick = 0;
static float s_drift_ref_roll = 0.0f;
static float s_drift_ref_pitch = 0.0f;
static float s_drift_ref_altitude = 0.0f;
static uint32_t s_drift_trip_count = 0;

/* Health tracking. */
static SensorHealthBlock_t s_health;

/*============================================================================*
 *                          INTERNAL HELPERS                                  *
 *============================================================================*/

/**
 * @brief Simple millisecond busy-wait. Replace with a TIM6 or SysTick
 *        delay once the protocol layer grows one.
 * @note  ms * 8000U can overflow uint32_t for ms > 536870.
 */
static void sensor_task_delay_ms(uint32_t ms) {
  for (volatile uint32_t i = 0U; i < ms * 8000U; i++) {
    __asm__("nop");
  }
}

/**
 * @brief Apply the per-IMU zero offsets to a raw reading, in place.
 */
static void apply_icm_bias(ICM20948_Data_t *d) {
  d->accel_x -= s_icm_accel_bias[0];
  d->accel_y -= s_icm_accel_bias[1];
  d->accel_z -= s_icm_accel_bias[2];
  d->gyro_x -= s_icm_gyro_bias[0];
  d->gyro_y -= s_icm_gyro_bias[1];
  d->gyro_z -= s_icm_gyro_bias[2];
}

static void apply_mpu_bias(MPU6050_Data_t *d) {
  d->AccelX -= s_mpu_accel_bias[0];
  d->AccelY -= s_mpu_accel_bias[1];
  d->AccelZ -= s_mpu_accel_bias[2];
  d->GyroX -= s_mpu_gyro_bias[0];
  d->GyroY -= s_mpu_gyro_bias[1];
  d->GyroZ -= s_mpu_gyro_bias[2];
}

/**
 * @brief Update the health block for one sensor's reading.
 */
static void health_update(SensorHealth_t *h, uint32_t *bad_ctr, bool ok) {
  if (ok) {
    /* Recovering takes a full window of good readings — this is what
     * makes the health flag a hysteresis, not a filter. */
    if (*bad_ctr > 0U) {
      (*bad_ctr)--;
    }
  } else {
    (*bad_ctr)++;
  }

  if (*bad_ctr >= 2U * SENSOR_TASK_HEALTH_SUSPECT_TH) {
    *h = SENSOR_HEALTH_FAILED;
  } else if (*bad_ctr >= SENSOR_TASK_HEALTH_SUSPECT_TH) {
    *h = SENSOR_HEALTH_SUSPECT;
  } else if (*h != SENSOR_HEALTH_DISAGREES) {
    /* Do not silently clear a DISAGREES flag from a good reading
     * alone — only an explicit recovery path (SensorTask_Recalibrate)
     * clears it. */
    *h = SENSOR_HEALTH_OK;
  }
}

/**
 * @brief Cross-verify the two IMUs' accelerometer readings.
 *
 * @details
 *   Both IMUs are on the same rigid body and mounted with aligned axes
 *   (per the assumption in sensor_fusion.h). At rest, both should read
 *   a ~1 g vector pointing the same way; under motion, both should see
 *   the same specific force. If they don't, at least one is wrong — we
 *   cannot tell which, so both get a strike.
 *
 *   Returns true if the pair agrees, false otherwise.
 */
static bool cross_verify_imus(const ICM20948_Data_t *icm,
                              const MPU6050_Data_t *mpu) {
  /* Magnitude comparison. */
  float icm_mag = sqrtf(icm->accel_x * icm->accel_x + icm->accel_y * icm->accel_y +
                        icm->accel_z * icm->accel_z);
  float mpu_mag = sqrtf(mpu->AccelX * mpu->AccelX + mpu->AccelY * mpu->AccelY +
                        mpu->AccelZ * mpu->AccelZ);

  if (fabsf(icm_mag - mpu_mag) > SENSOR_TASK_IMU_MAG_DISAGREE_G) {
    return false;
  }

  /* Axis-wise comparison. */
  if (fabsf(icm->accel_x - mpu->AccelX) > SENSOR_TASK_IMU_AXIS_DISAGREE_G)
    return false;
  if (fabsf(icm->accel_y - mpu->AccelY) > SENSOR_TASK_IMU_AXIS_DISAGREE_G)
    return false;
  if (fabsf(icm->accel_z - mpu->AccelZ) > SENSOR_TASK_IMU_AXIS_DISAGREE_G)
    return false;

  return true;
}

/**
 * @brief IMU still-start calibration.
 *
 * @details
 *   Averages SENSOR_TASK_CALIB_SAMPLES readings from each IMU and stores
 *   the mean as the per-IMU zero offset. Accel Z is special-cased to
 *   store (mean − 1 g), so the read path can uniformly subtract
 *   accel_bias[i] from all three axes.
 *
 *   Rejects the calibration (returns false) if any sample is out of the
 *   plausibility window — that usually means the board wasn't actually
 *   stationary.
 */
static bool calibrate_imus(void) {
  double icm_a[3] = {0.0, 0.0, 0.0};
  double icm_g[3] = {0.0, 0.0, 0.0};
  double mpu_a[3] = {0.0, 0.0, 0.0};
  double mpu_g[3] = {0.0, 0.0, 0.0};

  uint32_t accepted = 0U;
  uint32_t attempts = 0U;

  while (accepted < SENSOR_TASK_CALIB_SAMPLES &&
         attempts < SENSOR_TASK_CALIB_MAX_ATTEMPTS) {

    attempts++;

    ICM20948_Data_t icm_sample;
    MPU6050_Data_t mpu_sample;

    ICM20948_ReadAll(&icm_sample);
    (void)MPU6050_enumReadData(&mpu_sample);

    /* Plausibility check — reject samples that look like motion. */
    float a_mag_icm = sqrtf(icm_sample.accel_x * icm_sample.accel_x +
                            icm_sample.accel_y * icm_sample.accel_y +
                            icm_sample.accel_z * icm_sample.accel_z);
    float g_mag_icm = sqrtf(icm_sample.gyro_x * icm_sample.gyro_x +
                            icm_sample.gyro_y * icm_sample.gyro_y +
                            icm_sample.gyro_z * icm_sample.gyro_z);

    float a_mag_mpu = sqrtf(mpu_sample.AccelX * mpu_sample.AccelX +
                            mpu_sample.AccelY * mpu_sample.AccelY +
                            mpu_sample.AccelZ * mpu_sample.AccelZ);
    float g_mag_mpu = sqrtf(mpu_sample.GyroX * mpu_sample.GyroX +
                            mpu_sample.GyroY * mpu_sample.GyroY +
                            mpu_sample.GyroZ * mpu_sample.GyroZ);

    bool ok = (a_mag_icm > 0.75f && a_mag_icm < 1.25f) &&
              (a_mag_mpu > 0.75f && a_mag_mpu < 1.25f) &&
              (g_mag_icm < 15.0f) /* < ~15 dps */
              && (g_mag_mpu < 15.0f);

    if (!ok) {
      sensor_task_delay_ms(SENSOR_TASK_CALIB_SAMPLE_DELAY_MS);
      continue;
    }

    icm_a[0] += icm_sample.accel_x;
    icm_a[1] += icm_sample.accel_y;
    icm_a[2] += icm_sample.accel_z;
    icm_g[0] += icm_sample.gyro_x;
    icm_g[1] += icm_sample.gyro_y;
    icm_g[2] += icm_sample.gyro_z;

    mpu_a[0] += mpu_sample.AccelX;
    mpu_a[1] += mpu_sample.AccelY;
    mpu_a[2] += mpu_sample.AccelZ;
    mpu_g[0] += mpu_sample.GyroX;
    mpu_g[1] += mpu_sample.GyroY;
    mpu_g[2] += mpu_sample.GyroZ;

    accepted++;
    sensor_task_delay_ms(SENSOR_TASK_CALIB_SAMPLE_DELAY_MS);
  }

  if (accepted == 0U) {
    return false;
  }

  double inv = 1.0 / (double)accepted;

  s_icm_accel_bias[0] = (float)(icm_a[0] * inv);
  s_icm_accel_bias[1] = (float)(icm_a[1] * inv);
  /* Z stores (mean − 1g) so the read path uniformly subtracts. */
  s_icm_accel_bias[2] = (float)(icm_a[2] * inv) - 1.0f;

  s_icm_gyro_bias[0] = (float)(icm_g[0] * inv);
  s_icm_gyro_bias[1] = (float)(icm_g[1] * inv);
  s_icm_gyro_bias[2] = (float)(icm_g[2] * inv);

  s_mpu_accel_bias[0] = (float)(mpu_a[0] * inv);
  s_mpu_accel_bias[1] = (float)(mpu_a[1] * inv);
  s_mpu_accel_bias[2] = (float)(mpu_a[2] * inv) - 1.0f;

  s_mpu_gyro_bias[0] = (float)(mpu_g[0] * inv);
  s_mpu_gyro_bias[1] = (float)(mpu_g[1] * inv);
  s_mpu_gyro_bias[2] = (float)(mpu_g[2] * inv);

  return true;
}

/**
 * @brief Barometric ground reference.
 */
static bool calibrate_baro_reference(void) {
  double sum = 0.0;
  uint32_t n = 0U;

  for (uint32_t i = 0U; i < SENSOR_TASK_BARO_REF_SAMPLES; i++) {
    BMP388_Data d;
    BMP388_GetData(&s_bmp_calib, &d);
    if (d.pressure > 30000.0f && d.pressure < 110000.0f) {
      sum += (double)d.pressure;
      n++;
    }
    sensor_task_delay_ms(20U);
  }

  if (n == 0U) {
    return false;
  }
  s_ground_pressure_pa = (float)(sum / (double)n);
  return true;
}

/**
 * @brief Reset the drift monitor's reference points to the current state.
 */
static void drift_monitor_reset_reference(void) {
  s_drift_ref_roll = s_attitude.roll_deg;
  s_drift_ref_pitch = s_attitude.pitch_deg;
  s_drift_ref_altitude = s_altitude.altitude_m;
  s_drift_trip_count = 0U;
}

/**
 * @brief Run the drift check if it's due.
 *
 * @details
 *   Only meaningful when the vehicle is supposed to be stationary (the
 *   caller signals that via SensorTask_SetStationary). During flight
 *   the reference is continuously re-set, so the check is effectively
 *   suspended — motion is expected and is not drift.
 */
static void drift_monitor_update(void) {
  s_drift_tick++;

  /* Suspend: motion is expected. Keep the reference aligned with the
   * current state so that when the vehicle stops again, the first
   * check compares against where it stopped, not where it started. */
  if (!s_stationary) {
    drift_monitor_reset_reference();
    return;
  }

  /* Warm-up: give the EKF time to converge the gyro bias before
   * trusting the drift check. */
  if (s_drift_tick < SENSOR_TASK_DRIFT_WARMUP_TICKS) {
    return;
  }

  /* Only check on schedule. */
  if ((s_drift_tick % SENSOR_TASK_DRIFT_CHECK_INTERVAL) != 0U) {
    return;
  }

  float d_roll = fabsf(s_attitude.roll_deg - s_drift_ref_roll);
  float d_pitch = fabsf(s_attitude.pitch_deg - s_drift_ref_pitch);
  float d_alt = fabsf(s_altitude.altitude_m - s_drift_ref_altitude);

  bool tripped = (d_roll > SENSOR_TASK_DRIFT_ATTITUDE_DEG) ||
                 (d_pitch > SENSOR_TASK_DRIFT_ATTITUDE_DEG) ||
                 (d_alt > SENSOR_TASK_DRIFT_ALTITUDE_M);

  if (tripped) {
    s_drift_trip_count++;
    if (s_drift_trip_count >= SENSOR_TASK_DRIFT_TRIP_COUNT) {
      /* Sustained drift with the vehicle supposed to be still.
       * Mark both IMUs suspect; the drift monitor itself does not
       * decide which one is at fault. */
      s_health.icm_health = SENSOR_HEALTH_SUSPECT;
      s_health.mpu_health = SENSOR_HEALTH_SUSPECT;
    }
  } else {
    s_drift_trip_count = 0U;
  }

  /* The reference advances every check — we're looking for drift
   * within a single check interval, not cumulative drift from boot.
   * The cumulative case is caught by the health block instead. */
  s_drift_ref_roll = s_attitude.roll_deg;
  s_drift_ref_pitch = s_attitude.pitch_deg;
  s_drift_ref_altitude = s_altitude.altitude_m;
}

/*============================================================================*
 *                          PUBLIC API — INIT                                 *
 *============================================================================*/

SensorTask_Status_t SensorTask_Init(void) {
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  memset(&s_health, 0, sizeof(s_health));

  s_snapshot.calib_state = SENSOR_CALIB_IN_PROGRESS;
  s_calib_state = SENSOR_CALIB_IN_PROGRESS;

  /* ---- 1. Protocol bring-up (board-specific). ----
   *
   * @note This assumes main.c has already configured the clocks and SPI. */

  /* ---- 2. Sensor driver bring-up. ---- */
  bool icm_ok = (ICM20948_Init(ICM20948_ACCEL_FS_2G, ICM20948_GYRO_FS_250DPS) ==
                 ICM20948_OK);
  /* BMP388_Init returns 1 on success, 0 on failure. */
  bool bmp_ok = (BMP388_Init(&s_bmp_calib) != 0U);
  bool mpu_ok = (MPU6050_enumInit() == OK);

  /* GPS: bring up the UART and driver state, but a fix is not
   * required for Init() to succeed — the module can take 30 s to
   * get one, and the IMU fusion does not depend on GPS. */
  gps_bus_stm32_setup(PCLK1_HZ, 9600U);
  s_gps.bus_context = 0;
  s_gps.timeout_ms = 100U;
  s_gps.rx_ring = (struct GPS_RingBuffer *)&s_gps_ring;
  s_gps.nmea = (struct NMEA_Parser *)&s_gps_parser;
  bool gps_ok = (GPS_Init(&s_gps) == GPS_OK);

  s_health.icm_health = icm_ok ? SENSOR_HEALTH_UNKNOWN : SENSOR_HEALTH_FAILED;
  s_health.mpu_health = mpu_ok ? SENSOR_HEALTH_UNKNOWN : SENSOR_HEALTH_FAILED;
  s_health.bmp_health = bmp_ok ? SENSOR_HEALTH_UNKNOWN : SENSOR_HEALTH_FAILED;
  s_health.gps_health = gps_ok ? SENSOR_HEALTH_UNKNOWN : SENSOR_HEALTH_FAILED;

  if (!icm_ok || !bmp_ok || !mpu_ok) {
    s_calib_state = SENSOR_CALIB_FAILED;
    s_snapshot.calib_state = s_calib_state;
    s_initialized = true; /* still initialized, just degraded */
    return SENSOR_TASK_ERROR_INIT_FAILED;
  }

  /* ---- 3. IMU still-start calibration. ---- */
  if (!calibrate_imus()) {
    s_calib_state = SENSOR_CALIB_FAILED;
    s_snapshot.calib_state = s_calib_state;
    s_initialized = true;
    return SENSOR_TASK_ERROR_INIT_FAILED;
  }

  /* ---- 4. Barometric ground reference. ---- */
  (void)calibrate_baro_reference();
  /* A failed baro reference is non-fatal — altitude just reads
   * absolute rather than relative to the calibration point, and the
   * fusion layer's slow bias learner will pull it toward the truth. */

  /* ---- 5. Seed the fusion EKF. ---- */
  ICM20948_ReadAll(&s_icm);
  apply_icm_bias(&s_icm);
  if (SF_InitFromAccel(s_icm.accel_x, s_icm.accel_y, s_icm.accel_z) != SF_OK) {
    (void)SF_Reset(0);
  }

  if (s_ground_pressure_pa > 0.0f) {
    SF_AltitudeSetGroundRef(s_ground_pressure_pa);
  }

  /* ---- 6. Reset the drift monitor's reference. ---- */
  (void)SF_GetAttitude(0, &s_attitude);
  (void)SF_GetAltitude(0, &s_altitude);
  drift_monitor_reset_reference();
  s_drift_tick = 0U;

  s_calib_state = SENSOR_CALIB_COMPLETE;
  s_snapshot.calib_state = s_calib_state;
  s_initialized = true;

  return SENSOR_TASK_OK;
}

SensorTask_Status_t SensorTask_Recalibrate(void) {
  if (!s_initialized) {
    return SENSOR_TASK_ERROR_NOT_INITIALIZED;
  }

  s_calib_state = SENSOR_CALIB_IN_PROGRESS;
  s_snapshot.calib_state = s_calib_state;

  /* Re-read and re-average. Assumes the board is stationary — the
   * caller is responsible for that (see header). */
  if (!calibrate_imus()) {
    s_calib_state = SENSOR_CALIB_FAILED;
    s_snapshot.calib_state = s_calib_state;
    return SENSOR_TASK_ERROR_INIT_FAILED;
  }

  (void)calibrate_baro_reference();

  /* Re-seed the fusion filter — the gyro bias estimate from the
   * previous run is now meaningless since the underlying sensor
   * offsets just changed. */
  ICM20948_ReadAll(&s_icm);
  apply_icm_bias(&s_icm);
  (void)SF_Reset(0);
  (void)SF_InitFromAccel(s_icm.accel_x, s_icm.accel_y, s_icm.accel_z);

  if (s_ground_pressure_pa > 0.0f) {
    SF_AltitudeSetGroundRef(s_ground_pressure_pa);
  }

  /* Clear the DISAGREES latch so the system gets a clean slate. */
  s_health.icm_health = SENSOR_HEALTH_OK;
  s_health.mpu_health = SENSOR_HEALTH_OK;

  (void)SF_GetAttitude(0, &s_attitude);
  (void)SF_GetAltitude(0, &s_altitude);
  drift_monitor_reset_reference();
  s_drift_tick = 0U;

  s_calib_state = SENSOR_CALIB_COMPLETE;
  s_snapshot.calib_state = s_calib_state;
  return SENSOR_TASK_OK;
}

void SensorTask_SetStationary(bool stationary) {
  s_stationary = stationary;
  if (stationary) {
    /* Re-anchor the drift reference at the moment we stopped. */
    (void)SF_GetAttitude(0, &s_attitude);
    (void)SF_GetAltitude(0, &s_altitude);
    drift_monitor_reset_reference();
    s_drift_tick = 0U;
  }
}

/*============================================================================*
 *                          PUBLIC API — UPDATE                               *
 *============================================================================*/

SensorTask_Status_t SensorTask_Update(void) {
  if (!s_initialized) {
    return SENSOR_TASK_ERROR_NOT_INITIALIZED;
  }
  if (s_calib_state != SENSOR_CALIB_COMPLETE) {
    return SENSOR_TASK_ERROR_NOT_CALIBRATED;
  }

  SensorTask_Status_t return_status = SENSOR_TASK_OK;

  /* ---- 1. Read both IMUs. ---- */
  ICM20948_ReadAll(&s_icm);
  bool mpu_read_ok = (MPU6050_enumReadData(&s_mpu) == OK);
  (void)mpu_read_ok; /* the ICM20948 read path has no return code today */

  /* Apply the per-IMU zero offsets (from Init calibration). */
  apply_icm_bias(&s_icm);
  apply_mpu_bias(&s_mpu);

  /* ---- 2. Cross-verify. ---- */
  bool imus_agree = cross_verify_imus(&s_icm, &s_mpu);

  if (!imus_agree) {
    s_health.icm_disagree_count++;
    s_health.mpu_disagree_count++;
    /* Both are marked DISAGREES if the disagreement is sustained. */
    if (s_health.icm_disagree_count >= 2U * SENSOR_TASK_HEALTH_SUSPECT_TH) {
      s_health.icm_health = SENSOR_HEALTH_DISAGREES;
      s_health.mpu_health = SENSOR_HEALTH_DISAGREES;
    }
  } else {
    /* Decay the disagreement counters on agreement — a single
     * transient bump should not permanently mark the pair as
     * disagreeing. */
    if (s_health.icm_disagree_count > 0U)
      s_health.icm_disagree_count--;
    if (s_health.mpu_disagree_count > 0U)
      s_health.mpu_disagree_count--;
  }

  float icm_mag = sqrtf(s_icm.accel_x * s_icm.accel_x +
                        s_icm.accel_y * s_icm.accel_y +
                        s_icm.accel_z * s_icm.accel_z);
  bool icm_plausible = (icm_mag > 0.5f && icm_mag < 4.0f);

  health_update(&s_health.icm_health, &s_health.icm_bad_readings, icm_plausible);
  health_update(&s_health.mpu_health, &s_health.mpu_bad_readings, imus_agree);

  /* ---- 3. Attitude fusion. ----
   *
   * Only feed the EKF if the IMUs agree; otherwise let the gyro
   * prediction carry the filter through this tick. Skipping the
   * accel update for one tick is harmless — the covariance grows
   * slightly, the estimate stays smooth. */
  static const float kDegToRad = 3.14159265358979323846f / 180.0f;
  float gx_rad = s_icm.gyro_x * kDegToRad;
  float gy_rad = s_icm.gyro_y * kDegToRad;
  float gz_rad = s_icm.gyro_z * kDegToRad;

  SF_Status_t sf_st;

  if (imus_agree) {
    sf_st = SF_Update(0, gx_rad, gy_rad, gz_rad, s_icm.accel_x, s_icm.accel_y,
                      s_icm.accel_z, SENSOR_TASK_DT_S);
  } else {
    /* Predict only — no measurement update this tick. */
    sf_st = SF_PredictGyro(0, gx_rad, gy_rad, gz_rad, SENSOR_TASK_DT_S);
  }

  if (sf_st == SF_ERROR_DIVERGED) {
    s_health.fusion_diverge_count++;
    return_status = SENSOR_TASK_ERROR_FUSION_DIVERGED;
    /* Recover: reset the filter, keep the sensor task running. The
     * FSM is expected to notice the increased diverge count and
     * decide whether to land. */
    (void)SF_Reset(0);
    (void)SF_InitFromAccel(s_icm.accel_x, s_icm.accel_y, s_icm.accel_z);
  }

  (void)SF_GetAttitude(0, &s_attitude);

  /* ---- 4. Barometric read + altitude fusion. ---- */
  BMP388_GetData(&s_bmp_calib, &s_bmp);

  bool bmp_ok = (s_bmp.pressure > 30000.0f) && (s_bmp.pressure < 110000.0f);
  health_update(&s_health.bmp_health, &s_health.bmp_bad_readings, bmp_ok);

  if (bmp_ok) {
    /* Report altitude relative to the calibration ground reference. */
    float p_rel = s_bmp.pressure;
    (void)SF_AltitudeFeedBaro(0, p_rel, SENSOR_TASK_DT_S);
  }

  /* ---- 5. GPS read + altitude fusion. ---- */
  bool gps_read_ok = (GPS_ReadData(&s_gps, &s_gps_fix) == GPS_OK);

  bool gps_fix_valid =
      gps_read_ok && s_gps_fix.satellites_in_use >= SENSOR_TASK_GPS_MIN_SATS &&
      s_gps_fix.fix_quality >= SENSOR_TASK_GPS_MIN_FIX_QUALITY &&
      s_gps_fix.rmc_status_valid;

  health_update(&s_health.gps_health, &s_health.gps_bad_readings,
                gps_fix_valid);

  if (gps_fix_valid) {
    (void)SF_AltitudeFeedGPS(0, (float)s_gps_fix.altitude_m, true);
  }

  (void)SF_GetAltitude(0, &s_altitude);

  /* ---- 6. Drift monitor. ---- */
  drift_monitor_update();

  /* ---- 7. Publish the snapshot. ---- */

  s_snapshot.tick_ms = s_drift_tick; /* placeholder until TIM6 */
  s_snapshot.update_count++;
  s_snapshot.valid = (s_calib_state == SENSOR_CALIB_COMPLETE);

  /* Attitude. */
  s_snapshot.qw = s_attitude.qw;
  s_snapshot.qx = s_attitude.qx;
  s_snapshot.qy = s_attitude.qy;
  s_snapshot.qz = s_attitude.qz;
  s_snapshot.roll_deg = s_attitude.roll_deg;
  s_snapshot.pitch_deg = s_attitude.pitch_deg;
  s_snapshot.yaw_deg = s_attitude.yaw_deg;

  s_snapshot.gyro_bias_x_rps = s_attitude.gyro_bias_x;
  s_snapshot.gyro_bias_y_rps = s_attitude.gyro_bias_y;
  s_snapshot.gyro_bias_z_rps = s_attitude.gyro_bias_z;

  /* Body-frame rates and accel — from the ICM (the trusted primary). */
  s_snapshot.gyro_x_dps = s_icm.gyro_x;
  s_snapshot.gyro_y_dps = s_icm.gyro_y;
  s_snapshot.gyro_z_dps = s_icm.gyro_z;
  s_snapshot.accel_x_g = s_icm.accel_x;
  s_snapshot.accel_y_g = s_icm.accel_y;
  s_snapshot.accel_z_g = s_icm.accel_z;

  /* Altitude. */
  s_snapshot.altitude_m = s_altitude.altitude_m;
  s_snapshot.vertical_speed_m_s = s_altitude.vertical_velocity_m_s;

  /* Barometric raw. */
  s_snapshot.pressure_pa = s_bmp.pressure;
  s_snapshot.temperature_c = s_bmp.temperature;
  s_snapshot.baro_altitude_m = s_altitude.baro_altitude_m;

  /* GPS. */
  s_snapshot.gps_fix_valid = gps_fix_valid;
  s_snapshot.gps_fix_quality = (uint8_t)s_gps_fix.fix_quality;
  s_snapshot.gps_satellites = s_gps_fix.satellites_in_use;
  s_snapshot.gps_latitude_deg = s_gps_fix.latitude_deg;
  s_snapshot.gps_longitude_deg = s_gps_fix.longitude_deg;
  s_snapshot.gps_altitude_m = (float)s_gps_fix.altitude_m;
  s_snapshot.gps_speed_m_s = (float)(s_gps_fix.speed_knots * 0.514444);
  s_snapshot.gps_course_deg = (float)s_gps_fix.course_deg;

  /* Health / calibration. */
  s_snapshot.calib_state = s_calib_state;
  s_snapshot.health = s_health;

  /* Convenience flags. */
  s_snapshot.imu_trustworthy = (s_health.icm_health == SENSOR_HEALTH_OK ||
                                s_health.mpu_health == SENSOR_HEALTH_OK) &&
                               (s_health.fusion_diverge_count == 0U);

  s_snapshot.altitude_trustworthy = (s_health.bmp_health == SENSOR_HEALTH_OK);

  s_snapshot.position_trustworthy = gps_fix_valid;

  return return_status;
}

SensorTask_Status_t SensorTask_GetSnapshot(SensorSnapshot_t *out) {
  if (!s_initialized) {
    return SENSOR_TASK_ERROR_NOT_INITIALIZED;
  }
  if (out == NULL) {
    return SENSOR_TASK_ERROR_INVALID_PARAM;
  }
  *out = s_snapshot;
  return SENSOR_TASK_OK;
}