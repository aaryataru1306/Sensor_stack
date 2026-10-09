/**
 ******************************************************************************
 * @file    sensor_fusion.h
 * @brief   Public API for the sensor-fusion layer: a 7-state Error-State
 *          Kalman Filter that fuses the ICM20948 and MPU6050 into a single,
 *          drift-corrected attitude estimate.
 * @details
 *   The two IMUs are *redundant* for attitude — both are 6-axis, neither
 *   has a magnetometer. This layer treats the ICM20948 as the primary
 *   (better gyro noise) and the MPU6050 as a secondary that can be used
 *   for the accel update when the ICM's accel is unreliable, or as a
 *   cross-check. The public API below supports both modes.
 *
 *   The filter estimates attitude (quaternion) AND gyro bias online.
 *   This is what actually kills drift: a plain complementary filter has
 *   no way to distinguish "the board is slowly rotating" from "the gyro
 *   has a small bias," because both look like a nonzero gyro reading.
 *   An EKF can, because it also has an accel reference to compare against.
 *
 *   Yaw is inherently unobservable without a magnetometer. The filter
 *   will hold a heading, and that heading will drift at whatever the
 *   residual gyro-Z bias is. GPS course-over-ground can be fused in a
 *   later, second-stage filter when the vehicle is moving.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              STATUS CODES                                  *
 *============================================================================*/

typedef enum {
    SF_OK = 0,                    /**< Operation succeeded. */
    SF_ERROR_NOT_INITIALIZED,     /**< Called before SF_Init(). */
    SF_ERROR_INVALID_PARAM,       /**< Null pointer or out-of-range arg. */
    SF_ERROR_DIVERGED,            /**< Filter covariance lost positive-
                                       definiteness. Caller should call
                                       SF_Reset() and accept that the
                                       first few hundred ms are re-
                                       convergence. */
    SF_ERROR_STALE                /**< Update called after too long a gap
                                       (see SF_MAX_DT_S) — the covariance
                                       has been inflated; the next few
                                       updates will be noisy. Not fatal. */
} SF_Status_t;

/*============================================================================*
 *                              PUBLIC TYPES                                  *
 *============================================================================*/

/**
 * @brief Which IMU's accelerometer feeds the measurement update.
 *
 * @details
 *   Both IMUs sit on the same rigid body, so their accel readings should
 *   agree up to sensor noise and their respective mounting rotations.
 *   For simplicity this layer assumes they are mounted the **same way**
 *   (aligned axes). If your ICM and MPU are mounted differently, apply
 *   the rotation to one of them before calling SF_UpdateAccel_*().
 */
typedef enum {
    SF_ACCEL_SRC_ICM = 0,   /**< Use the ICM20948's accelerometer. */
    SF_ACCEL_SRC_MPU,       /**< Use the MPU6050's accelerometer. */
    SF_ACCEL_SRC_BLENDED    /**< Average the two (assuming aligned). */
} SF_AccelSource_t;

/**
 * @brief Fused attitude output, in the form most consumers actually want.
 *
 * @details
 *   Provides both the quaternion (for anything that wants to rotate
 *   vectors into world frame) and the Euler angles (for display,
 *   logging, and humans). Euler angles are computed from the quaternion
 *   on demand — they are not the filter state, so there is no
 *   gimbal-lock concern inside the filter itself, only in the
 *   conversion to Euler when pitch approaches ±90°.
 */
typedef struct {
    float qw, qx, qy, qz;   /**< Unit quaternion (body → world). */

    float roll_deg;         /**< Roll  [°], −180..+180. */
    float pitch_deg;        /**< Pitch [°],  −90..+90. */
    float yaw_deg;          /**< Yaw   [°],    0..360 (now absolute with mag). */

    float gyro_bias_x;      /**< Estimated gyro bias X [rad/s]. */
    float gyro_bias_y;      /**< Estimated gyro bias Y [rad/s]. */
    float gyro_bias_z;      /**< Estimated gyro bias Z [rad/s]. */

    uint32_t update_count;  /**< Total updates applied. */
    float    dt_last_s;     /**< Actual dt used on the last update [s]. */
} SF_Attitude_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Reset the filter to its initial state.
 *
 * @details
 *   Sets the quaternion to identity (no rotation — assumes the board is
 *   flat and level at the moment of this call), zeroes the estimated
 *   gyro bias, and reinitializes the covariance from the config defaults.
 *
 *   **Call this once, with the board at rest and roughly level.** If
 *   the board is not level when this is called, the filter will slowly
 *   converge to the truth over the next few seconds as the accel update
 *   pulls the attitude toward gravity — but there will be a visible
 *   transient. For an accurate initial attitude regardless of mounting,
 *   call SF_InitFromAccel() instead.
 *
 * @retval SF_OK                   Filter is reset and ready.
 * @retval SF_ERROR_INVALID_PARAM  ctx was NULL.
 */
SF_Status_t SF_Reset(void *ctx);

/**
 * @brief Initialize the filter using the current accelerometer reading
 *        as a gravity reference, so the initial attitude is correct
 *        regardless of how the board is mounted.
 *
 * @param[in] ax_g, ay_g, az_g  Accelerometer reading [g]. Must be a
 *                              *stationary* sample — the function treats
 *                              it as pure gravity.
 *
 * @retval SF_OK                   Initial attitude set from the accel.
 * @retval SF_ERROR_INVALID_PARAM  |a| outside the plausibility window
 *                                 (see SF_ACCEL_MAG_MIN/MAX).
 */
SF_Status_t SF_InitFromAccel(float ax_g, float ay_g, float az_g);

/**
 * @brief Predict step: propagate the attitude forward using a gyro
 *        sample and the current estimate of the gyro bias.
 *
 * @details
 *   Call this at the gyro's natural rate (typically 100–1000 Hz). The
 *   dt is supplied by the caller because the caller is the one with a
 *   real timer — using a nominal dt here would silently miscalibrate
 *   the whole filter on any jitter.
 *
 * @param[in] gx, gy, gz  Gyro reading [rad/s] from the primary IMU
 *                        (ICM20948 by convention). The function subtracts
 *                        the current bias estimate internally.
 * @param[in] dt_s        Time since the last predict or update [s].
 *
 * @retval SF_OK                   Predict applied.
 * @retval SF_ERROR_NOT_INITIALIZED  Filter was never reset.
 * @retval SF_ERROR_INVALID_PARAM  ctx NULL, dt_s ≤ 0 or dt_s > SF_MAX_DT_S.
 */
SF_Status_t SF_PredictGyro(void *ctx, float gx, float gy, float gz, float dt_s);

/**
 * @brief Measurement update: correct the attitude using an
 *        accelerometer reading (assumed to be dominated by gravity).
 *
 * @details
 *   Rejects the measurement outright if |a| is outside the plausibility
 *   window, or if the normalized innovation squared exceeds the
 *   Mahalanobis gate — in either case the state is left unchanged and
 *   the appropriate diagnostic counter incremented.
 *
 * @param[in] ax_g, ay_g, az_g  Accelerometer reading [g].
 *
 * @retval SF_OK                   Update applied (or cleanly rejected —
 *                                 rejection is not an error, see the
 *                                 diagnostic counters in SF_GetAttitude).
 * @retval SF_ERROR_NOT_INITIALIZED  Filter was never reset.
 * @retval SF_ERROR_INVALID_PARAM  ctx NULL, or the reading was rejected
 *                                 as an outlier.
 * @retval SF_ERROR_DIVERGED       The covariance matrix lost positive-
 *                                 definiteness. Caller should SF_Reset().
 */
SF_Status_t SF_UpdateAccel(void *ctx, float ax_g, float ay_g, float az_g);

/**
 * @brief Measurement update: correct the yaw using a magnetometer reading.
 *
 * @param[in] mx_g, my_g, mz_g  Magnetometer reading [Gauss].
 *
 * @retval SF_OK                   Update applied.
 * @retval SF_ERROR_NOT_INITIALIZED  Filter was never reset.
 * @retval SF_ERROR_INVALID_PARAM  ctx NULL.
 */
SF_Status_t SF_UpdateMag(void *ctx, float mx_g, float my_g, float mz_g);

/**
 * @brief Convenience: predict with gyro, then update with accel, in one
 *        call.
 *
 * @details
 *   The typical call shape for a main loop that reads both IMUs every
 *   tick. Uses the accelerometer source selected at compile time (see
 *   SF_ACCEL_SRC_* — the default is ICM).
 *
 * @param[in] gx, gy, gz  Gyro reading [rad/s] from the primary IMU.
 * @param[in] ax_g, ay_g, az_g  Accelerometer reading [g] from the
 *                              selected source.
 * @param[in] mx_g, my_g, mz_g  Magnetometer reading [Gauss].
 * @param[in] dt_s        Time since the last call [s].
 *
 * @retval SF_OK on success. See SF_PredictGyro and SF_UpdateAccel for
 *         the full status breakdown.
 */
SF_Status_t SF_Update(void *ctx,
                      float gx, float gy, float gz,
                      float ax_g, float ay_g, float az_g,
                      float mx_g, float my_g, float mz_g,
                      float dt_s);

/**
 * @brief Read the current fused attitude.
 *
 * @param[out] out  Filled in with the current estimate. Must not be NULL.
 *
 * @retval SF_OK                   Output populated.
 * @retval SF_ERROR_NOT_INITIALIZED  Filter was never reset.
 * @retval SF_ERROR_INVALID_PARAM  ctx or out NULL.
 */
SF_Status_t SF_GetAttitude(void *ctx, SF_Attitude_t *out);

/*============================================================================*
 *                          SECOND-STAGE FUSION                               *
 *============================================================================*
 *
 * Altitude (BMP388 + GPS) and position (GPS) are fused in a separate,
 * slower filter. Mixing them into the attitude EKF would make the
 * covariance matrix ill-conditioned, because the two sensor sets have
 * sample rates that differ by two orders of magnitude. The API below is
 * the two-stage design real flight stacks use.
 */

/**
 * @brief Fused altitude estimate, from BMP388 and GPS.
 */
typedef struct {
    float altitude_m;         /**< Fused altitude above the initial point [m]. */
    float vertical_velocity_m_s; /**< Fused climb rate [m/s]. */
    float pressure_pa;        /**< Last BMP388 pressure [Pa]. */
    float baro_altitude_m;    /**< BMP388-only altitude, for comparison. */
    float gps_altitude_m;     /**< Last GPS altitude [m], if any. */
    bool  gps_altitude_valid; /**< Whether the GPS altitude has ever been seen. */
    uint32_t baro_updates;    /**< Count of BMP388 updates applied. */
    uint32_t gps_updates;     /**< Count of GPS updates applied. */
} SF_Altitude_t;

/**
 * @brief Second-stage filter: BMP388 pressure → altitude, plus GPS
 *        altitude as a slow absolute reference.
 *
 * @details
 *   A 2-state (altitude, vertical velocity) linear Kalman filter. The
 *   BMP388 provides a high-rate but slowly-drifting measurement; the GPS
 *   provides a low-rate but absolutely-referenced measurement. The
 *   fusion removes the barometric drift without throwing away the
 *   barometer's good short-term accuracy.
 *
 *   The GPS altitude update is *only* applied when the GPS reports a
 *   valid 3D fix (see SF_AltitudeFeedGPS's return codes).
 *
 * @param[in] pressure_pa  BMP388 pressure [Pa].
 * @param[in] dt_s         Time since the last altitude call [s].
 *
 * @retval SF_OK on success.
 */
SF_Status_t SF_AltitudeFeedBaro(void *ctx, float pressure_pa, float dt_s);

/**
 * @brief Feed a GPS position/altitude sample into the altitude filter.
 *
 * @param[in] gps_altitude_m  GPS-reported altitude [m].
 * @param[in] valid           True if the GPS fix has a valid 3D altitude
 *                            (i.e. the GGA fix quality is DGPS or better
 *                            and the sentence included an altitude).
 *
 * @retval SF_OK on success.
 */
SF_Status_t SF_AltitudeFeedGPS(void *ctx, float gps_altitude_m, bool valid);

/**
 * @brief Seed the altitude filter with a barometric ground reference.
 *
 * @param[in] pressure_pa  BMP388 ground reference pressure [Pa].
 */
void SF_AltitudeSetGroundRef(float pressure_pa);

/**
 * @brief Read the fused altitude state.
 */
SF_Status_t SF_GetAltitude(void *ctx, SF_Altitude_t *out);

#ifdef __cplusplus
}
#endif


/* ============================================================================ *
 *                       INTEGRATED SENSOR FUSION CONFIG                        *
 * ============================================================================ */

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


/* ============================================================================ *
 *                       INTEGRATED SENSOR FUSION PRIVATE                       *
 * ============================================================================ */

/**
 ******************************************************************************
 * @file    sensor_fusion_private.h
 * @brief   Internal EKF state, quaternion math helpers, and matrix ops —
 *          sensor-fusion-layer internal.
 * @details
 *   Nothing in this file is part of the public API. Only sensor_fusion.c
 *   should include it.
 ******************************************************************************
 */


#include <stdint.h>
#include <math.h>

/*============================================================================*
 *                          STATE VECTOR LAYOUT                               *
 *============================================================================*/

/** @brief Number of states in the EKF: 4 quaternion + 3 gyro bias. */
#define SF_N_STATES     7U

/** @brief Index of the first quaternion component (qw). */
#define SF_IDX_QW       0U
#define SF_IDX_QX       1U
#define SF_IDX_QY       2U
#define SF_IDX_QZ       3U

/** @brief Index of the first gyro-bias component (bgx). */
#define SF_IDX_BGX      4U
#define SF_IDX_BGY      5U
#define SF_IDX_BGZ      6U

/*============================================================================*
 *                          FIXED-SIZE MATRIX OPS                             *
 *============================================================================*
 *
 * There is no BLAS on a bare-metal Cortex-M4 that we want to link, and
 * the matrices here are small and fixed-size. Everything below operates
 * on stack-allocated arrays of known dimensions, so the compiler can
 * unroll and register-allocate them. This is what makes the EKF run in
 * well under 1 ms on a 100 MHz M4 with FPU.
 */

/** @brief Fixed 7×7 matrix, row-major. */
typedef struct { float m[SF_N_STATES][SF_N_STATES]; } sf_mat7_t;

/** @brief Fixed 7-vector. */
typedef struct { float v[SF_N_STATES]; } sf_vec7_t;

/*============================================================================*
 *                          QUATERNION HELPERS                                *
 *============================================================================*/

/**
 * @brief Normalize a quaternion in place.
 * @param[in,out] q  Quaternion as [w, x, y, z].
 */
void sf_quat_normalize(float q[4]);

/**
 * @brief Convert a quaternion to a 3×3 rotation matrix (body → world).
 * @param[in]  q     Quaternion as [w, x, y, z].
 * @param[out] R_out Row-major 3×3 matrix, R_out[9].
 */
void sf_quat_to_rotmat(const float q[4], float R_out[9]);

/**
 * @brief Convert a small rotation vector to a quaternion increment.
 * @details Uses the small-angle approximation: for |θ| < ~10°, the
 *          quaternion is (1, θ/2) normalized. This is the standard
 *          first-order update and is what keeps the EKF's attitude
 *          propagation cheap.
 *
 * @param[in]  dtheta  Rotation vector [rad] as [x, y, z].
 * @param[out] dq      Quaternion increment [w, x, y, z].
 */
void sf_quat_from_small_rot(const float dtheta[3], float dq[4]);

/**
 * @brief Hamilton product: q_out = q_a ⊗ q_b.
 */
void sf_quat_multiply(const float q_a[4], const float q_b[4], float q_out[4]);

/*============================================================================*
 *                          MATRIX OPS (7×7 and mixed)                        *
 *============================================================================*/

void sf_mat7_identity(sf_mat7_t *A);
void sf_mat7_zero(sf_mat7_t *A);
void sf_mat7_copy(const sf_mat7_t *src, sf_mat7_t *dst);
void sf_mat7_add(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C);
void sf_mat7_sub(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C);
void sf_mat7_mul(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C);
void sf_mat7_transpose(const sf_mat7_t *A, sf_mat7_t *At);
void sf_mat7_scale(sf_mat7_t *A, float s);

/** @brief Symmetrize P in place: P = (P + Pᵀ)/2. Called after every
 *         update to stop numerical asymmetry from accumulating. */
void sf_mat7_symmetrize(sf_mat7_t *P);

/** @brief In-place Cholesky-based inverse of a symmetric positive-
 *         definite 7×7 matrix. Returns 0 on success, -1 if P is not
 *         positive definite (which should never happen with correct Q/R,
 *         but the return code lets the caller skip the update instead of
 *         propagating NaNs).
 */
int sf_mat7_inverse_spd(const sf_mat7_t *A, sf_mat7_t *Ainv);

/*============================================================================*
 *                          FUSION INTERNAL STATE                             *
 *============================================================================*/

/**
 * @brief Full internal state of one EKF instance.
 *
 * @details
 *   Opaque to callers — the public handle wraps a pointer to this. Kept
 *   separate from the public header so the 7×7 covariance matrix doesn't
 *   appear in every translation unit that includes sensor_fusion.h.
 */
typedef struct {
    /* --- State --- */
    float q[4];                        /**< Attitude quaternion. */
    float bg[3];                       /**< Gyro bias [rad/s]. */

    /* --- Covariance --- */
    sf_mat7_t P;

    /* --- Process noise --- */
    sf_mat7_t Q;

    /* --- Last update time --- */
    float last_dt_s;

    /* --- Diagnostics --- */
    uint32_t updates_gyro;
    uint32_t updates_accel;
    uint32_t rejections_magnitude;
    uint32_t rejections_mahalanobis;
    int      last_init_ok;
} sf_internal_t;

#endif /* SENSOR_FUSION_H */
