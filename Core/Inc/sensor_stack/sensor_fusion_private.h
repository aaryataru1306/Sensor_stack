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

#ifndef SENSOR_FUSION_PRIVATE_H
#define SENSOR_FUSION_PRIVATE_H

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

#endif /* SENSOR_FUSION_PRIVATE_H */