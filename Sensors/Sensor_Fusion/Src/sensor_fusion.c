/**
 ******************************************************************************
 * @file    sensor_fusion.c
 * @brief   Implementation of the sensor-fusion EKF declared in
 *          sensor_fusion.h.
 * @details
 *   This file is deliberately self-contained: no dependency on the
 *   protocol layer, no dependency on any sensor driver, no HAL. It
 *   takes plain floating-point inputs and produces plain floating-point
 *   outputs. That makes it testable on a host PC against recorded
 *   sensor data, which is the only sane way to tune an EKF.
 *
 *   The filter is a 7-state Error-State Kalman Filter:
 *     - 4 states: attitude quaternion (body → world)
 *     - 3 states: gyro bias in body frame
 *
 *   Prediction uses the gyro; measurement update uses the accelerometer.
 *   Yaw is unobservable and will drift at the residual gyro-Z bias rate
 *   — that is a physical property of a 6-axis IMU, not a bug.
 *
 *   A second, independent 2-state linear filter fuses BMP388 pressure
 *   and GPS altitude into a single drift-corrected altitude estimate.
 *   It shares the file but not the state — see SF_AltitudeFeedBaro and
 *   friends.
 ******************************************************************************
 */

#include "sensor_fusion.h"
#include "sensor_fusion_private.h"
#include "sensor_fusion_config.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_PER_RAD (180.0f / M_PI)
#define RAD_PER_DEG (M_PI / 180.0f)

/*============================================================================*
 *                          INSTANCE STORAGE                                  *
 *============================================================================*
 *
 * The public API is void* to hide the 7×7 covariance matrix from every
 * caller. The actual storage lives here, in one file-static instance.
 * If you ever need two independent filters (say one per IMU, cross-
 * checked against each other), give SF_Reset() a pointer-to-storage
 * parameter and duplicate this struct — the math is all instance-safe.
 */

static sf_internal_t g_sf;

/*============================================================================*
 *                          QUATERNION MATH                                   *
 *============================================================================*/

void sf_quat_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-12f) {
        /* Degenerate — fall back to identity rather than dividing by ~0. */
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void sf_quat_to_rotmat(const float q[4], float R[9])
{
    /* Standard quaternion-to-rotation-matrix, orthonormalized. */
    float ww = q[0]*q[0], xx = q[1]*q[1], yy = q[2]*q[2], zz = q[3]*q[3];
    float wx = q[0]*q[1], wy = q[0]*q[2], wz = q[0]*q[3];
    float xy = q[1]*q[2], xz = q[1]*q[3], yz = q[2]*q[3];

    R[0] = ww + xx - yy - zz;  R[1] = 2.0f*(xy - wz);        R[2] = 2.0f*(xz + wy);
    R[3] = 2.0f*(xy + wz);     R[4] = ww - xx + yy - zz;     R[5] = 2.0f*(yz - wx);
    R[6] = 2.0f*(xz - wy);     R[7] = 2.0f*(yz + wx);        R[8] = ww - xx - yy + zz;
}

void sf_quat_from_small_rot(const float dtheta[3], float dq[4])
{
    /* First-order: dq ≈ (1, θ/2), normalized. Exact enough for the
     * sub-degree increments a 100 Hz gyro produces. */
    dq[0] = 1.0f;
    dq[1] = 0.5f * dtheta[0];
    dq[2] = 0.5f * dtheta[1];
    dq[3] = 0.5f * dtheta[2];
    sf_quat_normalize(dq);
}

void sf_quat_multiply(const float qa[4], const float qb[4], float qo[4])
{
    /* Hamilton product, expanded. */
    qo[0] = qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3];
    qo[1] = qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2];
    qo[2] = qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1];
    qo[3] = qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0];
}

/*============================================================================*
 *                          MATRIX OPS (7×7)                                  *
 *============================================================================*/

void sf_mat7_identity(sf_mat7_t *A)
{
    memset(A, 0, sizeof(*A));
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        A->m[i][i] = 1.0f;
    }
}

void sf_mat7_zero(sf_mat7_t *A) { memset(A, 0, sizeof(*A)); }

void sf_mat7_copy(const sf_mat7_t *src, sf_mat7_t *dst)
{
    memcpy(dst, src, sizeof(*dst));
}

void sf_mat7_add(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C)
{
    for (uint32_t i = 0; i < SF_N_STATES; i++)
        for (uint32_t j = 0; j < SF_N_STATES; j++)
            C->m[i][j] = A->m[i][j] + B->m[i][j];
}

void sf_mat7_sub(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C)
{
    for (uint32_t i = 0; i < SF_N_STATES; i++)
        for (uint32_t j = 0; j < SF_N_STATES; j++)
            C->m[i][j] = A->m[i][j] - B->m[i][j];
}

void sf_mat7_mul(const sf_mat7_t *A, const sf_mat7_t *B, sf_mat7_t *C)
{
    /* Use a local accumulator so A, B, C can alias safely. */
    sf_mat7_t tmp;
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j < SF_N_STATES; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < SF_N_STATES; k++) {
                s += A->m[i][k] * B->m[k][j];
            }
            tmp.m[i][j] = s;
        }
    }
    *C = tmp;
}

void sf_mat7_transpose(const sf_mat7_t *A, sf_mat7_t *At)
{
    sf_mat7_t tmp;
    for (uint32_t i = 0; i < SF_N_STATES; i++)
        for (uint32_t j = 0; j < SF_N_STATES; j++)
            tmp.m[i][j] = A->m[j][i];
    *At = tmp;
}

void sf_mat7_scale(sf_mat7_t *A, float s)
{
    for (uint32_t i = 0; i < SF_N_STATES; i++)
        for (uint32_t j = 0; j < SF_N_STATES; j++)
            A->m[i][j] *= s;
}

void sf_mat7_symmetrize(sf_mat7_t *P)
{
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = i + 1U; j < SF_N_STATES; j++) {
            float avg = 0.5f * (P->m[i][j] + P->m[j][i]);
            P->m[i][j] = avg;
            P->m[j][i] = avg;
        }
    }
}

int sf_mat7_inverse_spd(const sf_mat7_t *A, sf_mat7_t *Ainv)
{
    /* Cholesky: A = L Lᵀ. If any diagonal of L goes non-positive, A
     * isn't positive-definite and the caller should skip the update. */
    float L[SF_N_STATES][SF_N_STATES] = {{0.0f}};
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j <= i; j++) {
            float sum = A->m[i][j];
            for (uint32_t k = 0; k < j; k++) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                if (sum <= 1e-12f) {
                    return -1;   /* not positive-definite */
                }
                L[i][j] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    /* Solve L Lᵀ X = I for X, column by column. Only the lower triangle
     * of each column needs to be solved for, then the upper triangle
     * mirrors it (A is symmetric). */
    sf_mat7_t tmp;
    sf_mat7_zero(&tmp);
    for (uint32_t col = 0; col < SF_N_STATES; col++) {
        float y[SF_N_STATES];
        /* Forward solve L y = e_col */
        for (uint32_t i = 0; i < SF_N_STATES; i++) {
            float sum = (i == col) ? 1.0f : 0.0f;
            for (uint32_t k = 0; k < i; k++) {
                sum -= L[i][k] * y[k];
            }
            y[i] = sum / L[i][i];
        }
        /* Back solve Lᵀ x = y */
        float x[SF_N_STATES];
        for (int i = (int)SF_N_STATES - 1; i >= 0; i--) {
            float sum = y[i];
            for (uint32_t k = (uint32_t)i + 1U; k < SF_N_STATES; k++) {
                sum -= L[k][i] * x[k];
            }
            x[i] = sum / L[i][i];
        }
        for (uint32_t i = 0; i < SF_N_STATES; i++) {
            tmp.m[i][col] = x[i];
        }
    }

    *Ainv = tmp;
    return 0;
}

/*============================================================================*
 *                          PRIVATE HELPERS                                   *
 *============================================================================*/

/**
 * @brief Clear the filter to a known initial state, quaternion identity.
 */
static void sf_reset_internal(void)
{
    g_sf.q[0] = 1.0f; g_sf.q[1] = 0.0f; g_sf.q[2] = 0.0f; g_sf.q[3] = 0.0f;
    g_sf.bg[0] = 0.0f; g_sf.bg[1] = 0.0f; g_sf.bg[2] = 0.0f;

    /* Initial covariance: small on attitude, larger on bias (we have no
     * idea what the bias is yet). */
    sf_mat7_zero(&g_sf.P);
    for (uint32_t i = 0; i < 4; i++) {
        g_sf.P.m[i][i] = SF_INIT_P_ATTITUDE;
    }
    for (uint32_t i = 4; i < SF_N_STATES; i++) {
        g_sf.P.m[i][i] = SF_INIT_P_GYRO_BIAS;
    }

    /* Process noise, constant across updates in this simplified design. */
    sf_mat7_zero(&g_sf.Q);
    for (uint32_t i = 0; i < 4; i++) {
        g_sf.Q.m[i][i] = SF_GYRO_NOISE_VAR;
    }
    for (uint32_t i = 4; i < SF_N_STATES; i++) {
        g_sf.Q.m[i][i] = SF_GYRO_BIAS_WALK_VAR;
    }

    g_sf.last_dt_s = SF_IMU_DT;
    g_sf.updates_gyro = 0U;
    g_sf.updates_accel = 0U;
    g_sf.rejections_magnitude = 0U;
    g_sf.rejections_mahalanobis = 0U;
    g_sf.last_init_ok = 1;
}

/*============================================================================*
 *                          PUBLIC API IMPLEMENTATION                         *
 *============================================================================*/

SF_Status_t SF_Reset(void *ctx)
{
    (void)ctx;   /* single-instance; ctx is a placeholder for a future
                    multi-instance API */
    sf_reset_internal();
    return SF_OK;
}

SF_Status_t SF_InitFromAccel(float ax_g, float ay_g, float az_g)
{
    float mag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (mag < SF_ACCEL_MAG_MIN || mag > SF_ACCEL_MAG_MAX) {
        return SF_ERROR_INVALID_PARAM;
    }

    /* The accel measures gravity in body frame. In world frame,
     * gravity points straight down: g_world = (0, 0, +1) in the
     * convention where Z is up and "1" is the normalized gravity
     * magnitude. We need a rotation q such that R(q) * g_body = g_world.
     *
     * The clean way to build this is: find the axis of rotation that
     * takes the normalized body-frame "up" vector to world "up", and
     * the angle between them. */
    float an = 1.0f / mag;
    float up_b[3] = { ax_g * an, ay_g * an, az_g * an };
    float up_w[3] = { 0.0f, 0.0f, 1.0f };

    /* Cross product and dot product of up_b × up_w. */
    float c[3] = {
        up_b[1]*up_w[2] - up_b[2]*up_w[1],
        up_b[2]*up_w[0] - up_b[0]*up_w[2],
        up_b[0]*up_w[1] - up_b[1]*up_w[0]
    };
    float d = up_b[0]*up_w[0] + up_b[1]*up_w[1] + up_b[2]*up_w[2];
    float s = sqrtf(c[0]*c[0] + c[1]*c[1] + c[2]*c[2]);

    sf_reset_internal();

    if (s < 1e-6f) {
        /* Vectors are parallel or anti-parallel — no rotation, or a
         * 180° flip about any axis. Identity for the parallel case. */
        if (d < 0.0f) {
            /* Anti-parallel: 180° about X. */
            g_sf.q[0] = 0.0f; g_sf.q[1] = 1.0f;
            g_sf.q[2] = 0.0f; g_sf.q[3] = 0.0f;
        }
        return SF_OK;
    }

    /* q = (cos(θ/2), axis * sin(θ/2)), with cos(θ/2) and sin(θ/2) built
     * from dot and cross magnitudes so we don't need an acos(). */
    float half_cos = sqrtf(0.5f * (1.0f + d));
    float half_sin = s * 0.5f / half_cos;   /* since sinθ = s, cosθ = d */
    float axis_n = 1.0f / s;

    g_sf.q[0] = half_cos;
    g_sf.q[1] = c[0] * axis_n * half_sin;
    g_sf.q[2] = c[1] * axis_n * half_sin;
    g_sf.q[3] = c[2] * axis_n * half_sin;

    sf_quat_normalize(g_sf.q);
    return SF_OK;
}

SF_Status_t SF_PredictGyro(void *ctx, float gx, float gy, float gz, float dt_s)
{
    (void)ctx;
    if (dt_s <= 0.0f || dt_s > SF_MAX_DT_S) {
        return SF_ERROR_INVALID_PARAM;
    }

    /* Bias-corrected angular rate in body frame. */
    float wx = gx - g_sf.bg[0];
    float wy = gy - g_sf.bg[1];
    float wz = gz - g_sf.bg[2];

    /* Small-angle quaternion increment and rotation compose. */
    float dtheta[3] = { wx * dt_s, wy * dt_s, wz * dt_s };
    float dq[4];
    sf_quat_from_small_rot(dtheta, dq);

    float q_new[4];
    sf_quat_multiply(g_sf.q, dq, q_new);
    memcpy(g_sf.q, q_new, sizeof(g_sf.q));
    sf_quat_normalize(g_sf.q);

    /* ---- Covariance propagation ----
     *
     * P = F P Fᵀ + Q, where F is the state transition Jacobian. For the
     * error-state formulation with small dt, F is approximately:
     *
     *     F = [ I₄    −(dt/2) * Ω ]      (top-left 4×4, top-right 4×3)
     *         [ 0₃ₓ₄      I₃       ]      (bottom)
     *
     * where Ω is a 4×3 matrix that maps the bias-error vector into the
     * attitude-error rate. Deriving and coding that sparse F explicitly
     * is more code than it's worth for a first cut — the small-angle
     * approximation means F ≈ I + dt*G, and the leading-order effect on
     * P is captured by just inflating P by the process noise Q. This is
     * what most lightweight EKF implementations do at this update rate,
     * and it's what gives the filter its numerical stability. */
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        g_sf.P.m[i][i] += g_sf.Q.m[i][i];
    }

    /* Bias random-walk: the bias itself doesn't change from gyro alone,
     * only its uncertainty grows. */
    g_sf.last_dt_s = dt_s;
    g_sf.updates_gyro++;
    return SF_OK;
}

SF_Status_t SF_UpdateAccel(void *ctx, float ax_g, float ay_g, float az_g)
{
    (void)ctx;

    /* ---- Magnitude gate ----
     * If the vehicle is accelerating, the accel isn't a clean gravity
     * reference. Reject rather than corrupt the attitude. */
    float mag2 = ax_g*ax_g + ay_g*ay_g + az_g*az_g;
    if (mag2 < SF_ACCEL_MAG_MIN * SF_ACCEL_MAG_MIN
     || mag2 > SF_ACCEL_MAG_MAX * SF_ACCEL_MAG_MAX) {
        g_sf.rejections_magnitude++;
        return SF_ERROR_INVALID_PARAM;
    }

    /* ---- Predicted gravity in body frame ----
     * R(q)ᵀ * (0, 0, 1) gives the world-frame up vector expressed in
     * body coordinates — which is what the accelerometer should be
     * reading (up to sign convention). */
    float R[9];
    sf_quat_to_rotmat(g_sf.q, R);
    /* Predicted body-frame gravity: third ROW of R (since Rᵀ * z_world
     * picks out the third row when z_world = (0,0,1)). */
    float g_pred[3] = { R[6], R[7], R[8] };
    float mag_n = 1.0f / sqrtf(mag2);
    float a_n[3] = { ax_g * mag_n, ay_g * mag_n, az_g * mag_n };

    /* ---- Measurement Jacobian H ----
     * H = ∂g_pred / ∂(attitude error). For a small attitude error δθ,
     * the change in predicted body-frame gravity is:
     *     δg_pred ≈ [g_pred]ₓ δθ
     * where [g_pred]ₓ is the skew-symmetric matrix of g_pred.
     *
     * The 3×7 H matrix has that 3×3 block in the attitude columns and
     * zeros in the bias columns (gravity doesn't depend on bias). */
    float H[3][SF_N_STATES] = {{0.0f}};
    H[0][SF_IDX_QW] = 0.0f;
    H[0][SF_IDX_QX] = 0.0f;                 H[0][SF_IDX_QY] = -g_pred[2];  H[0][SF_IDX_QZ] =  g_pred[1];
    H[1][SF_IDX_QX] = g_pred[2];            H[1][SF_IDX_QY] = 0.0f;       H[1][SF_IDX_QZ] = -g_pred[0];
    H[2][SF_IDX_QX] = -g_pred[1];           H[2][SF_IDX_QY] = g_pred[0];   H[2][SF_IDX_QZ] = 0.0f;
    /* Columns 4..6 (bias) are zero. */

    /* ---- Innovation ----
     * Innovation y = a_n − g_pred. */
    float y[3] = { a_n[0] - g_pred[0],
                   a_n[1] - g_pred[1],
                   a_n[2] - g_pred[2] };

    /* ---- S = H P Hᵀ + R ----
     * R is a scalar on each accel axis, so R = SF_ACCEL_NOISE_VAR * I₃. */
    float HP[3][SF_N_STATES];
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t j = 0; j < SF_N_STATES; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < SF_N_STATES; k++) {
                s += H[i][k] * g_sf.P.m[k][j];
            }
            HP[i][j] = s;
        }
    }
    float S[3][3];
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < SF_N_STATES; k++) {
                s += HP[i][k] * H[j][k];   /* H[j][k], not H[k][j] */
            }
            S[i][j] = s + ((i == j) ? SF_ACCEL_NOISE_VAR : 0.0f);
        }
    }

    /* ---- Mahalanobis gate: yᵀ S⁻¹ y ≤ threshold? ----
     * Invert the 3×3 S explicitly (closed-form for a 3×3). */
    float det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
              - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
              + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    if (fabsf(det) < 1e-12f) {
        return SF_ERROR_INVALID_PARAM;
    }
    float inv_det = 1.0f / det;
    float Sinv[3][3];
    Sinv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    Sinv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * inv_det;
    Sinv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    Sinv[1][0] = -(S[1][0]*S[2][2] - S[1][2]*S[2][0]) * inv_det;
    Sinv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    Sinv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * inv_det;
    Sinv[2][0] =  (S[1][0]*S[2][1] - S[1][1]*S[2][0]) * inv_det;
    Sinv[2][1] = -(S[0][0]*S[2][1] - S[0][1]*S[2][0]) * inv_det;
    Sinv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;

    float mahal = 0.0f;
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 3; j++)
            mahal += y[i] * Sinv[i][j] * y[j];

    if (mahal > SF_ACCEL_MAHALANOBIS_GATE) {
        g_sf.rejections_mahalanobis++;
        return SF_ERROR_INVALID_PARAM;
    }

    /* ---- Kalman gain K = P Hᵀ S⁻¹ ----
     * P Hᵀ is 7×3 (transpose of HP basically, but computed as P * Hᵀ). */
    float PHt[SF_N_STATES][3];
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < SF_N_STATES; k++) {
                s += g_sf.P.m[i][k] * H[j][k];
            }
            PHt[i][j] = s;
        }
    }
    float K[SF_N_STATES][3];
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < 3; k++) {
                s += PHt[i][k] * Sinv[k][j];
            }
            K[i][j] = s;
        }
    }

    /* ---- State update ----
     * Attitude update: q ← q ⊗ small_rot(K_attitude * y).
     * Bias update:     bg ← bg + K_bias * y. */
    float dtheta[3] = { 0.0f, 0.0f, 0.0f };
    for (uint32_t i = 0; i < 3; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            dtheta[i] += K[SF_IDX_QX + i][j] * y[j];
        }
    }
    float dq[4];
    sf_quat_from_small_rot(dtheta, dq);
    float q_new[4];
    sf_quat_multiply(g_sf.q, dq, q_new);
    memcpy(g_sf.q, q_new, sizeof(g_sf.q));
    sf_quat_normalize(g_sf.q);

    for (uint32_t i = 0; i < 3; i++) {
        float db = 0.0f;
        for (uint32_t j = 0; j < 3; j++) {
            db += K[SF_IDX_BGX + i][j] * y[j];
        }
        g_sf.bg[i] += db;

        /* Clamp bias to a physically plausible range, guarding against
         * a run of corrupted measurements winding it up to nonsense. */
        if (g_sf.bg[i] >  SF_GYRO_BIAS_MAX) g_sf.bg[i] =  SF_GYRO_BIAS_MAX;
        if (g_sf.bg[i] < -SF_GYRO_BIAS_MAX) g_sf.bg[i] = -SF_GYRO_BIAS_MAX;
    }

    /* ---- Covariance update ----
     * P ← (I − K H) P, with the Joseph form for numerical stability:
     * P ← (I−KH) P (I−KH)ᵀ + K R Kᵀ.
     *
     * The Joseph form is used because the simple form loses symmetry
     * and can go non-positive-definite after enough updates. The cost
     * is a few more multiplies; the benefit is a filter that doesn't
     * need a reset after a day of running. */
    sf_mat7_t IKH;
    sf_mat7_identity(&IKH);
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j < SF_N_STATES; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < 3; k++) {
                s += K[i][k] * H[k][j];
            }
            IKH.m[i][j] -= s;
        }
    }
    sf_mat7_t tmp1, tmp2;
    sf_mat7_mul(&IKH, &g_sf.P, &tmp1);
    sf_mat7_t IKH_t;
    sf_mat7_transpose(&IKH, &IKH_t);
    sf_mat7_mul(&tmp1, &IKH_t, &tmp2);
    /* Now add K R Kᵀ (R is scalar, so it's R * K Kᵀ). */
    sf_mat7_t KKT = {{{{0}}}};
    for (uint32_t i = 0; i < SF_N_STATES; i++) {
        for (uint32_t j = 0; j < SF_N_STATES; j++) {
            float s = 0.0f;
            for (uint32_t k = 0; k < 3; k++) {
                s += K[i][k] * K[j][k];
            }
            KKT.m[i][j] = s * SF_ACCEL_NOISE_VAR;
        }
    }
    sf_mat7_add(&tmp2, &KKT, &g_sf.P);
    sf_mat7_symmetrize(&g_sf.P);

    g_sf.updates_accel++;
    return SF_OK;
}

SF_Status_t SF_UpdateMag(void *ctx, float mx_g, float my_g, float mz_g)
{
    (void)ctx;
    
    float mag2 = mx_g*mx_g + my_g*my_g + mz_g*mz_g;
    if (mag2 < 1e-6f) return SF_ERROR_INVALID_PARAM;

    /* A robust Complementary Filter approach to yaw correction. 
       This avoids destabilizing the 7-state EKF covariance matrix. */
    float R[9];
    sf_quat_to_rotmat(g_sf.q, R);
    
    /* Project the magnetometer reading into the horizontal world plane */
    float hx = mx_g * R[0] + my_g * R[1] + mz_g * R[2];
    float hy = mx_g * R[3] + my_g * R[4] + mz_g * R[5];
    
    /* The error in yaw between current heading and magnetic North */
    float err_yaw = atan2f(hy, hx);
    
    /* Apply a small gain correction step towards Magnetic North */
    float gain = 0.01f; 
    float dtheta[3] = { 0.0f, 0.0f, err_yaw * gain };
    
    float dq[4];
    sf_quat_from_small_rot(dtheta, dq);
    
    float q_new[4];
    /* Multiply dq * q to apply yaw rotation in the world frame */
    sf_quat_multiply(dq, g_sf.q, q_new); 
    memcpy(g_sf.q, q_new, sizeof(g_sf.q));
    sf_quat_normalize(g_sf.q);
    
    return SF_OK;
}

SF_Status_t SF_Update(void *ctx,
                      float gx, float gy, float gz,
                      float ax_g, float ay_g, float az_g,
                      float mx_g, float my_g, float mz_g,
                      float dt_s)
{
    SF_Status_t st = SF_PredictGyro(ctx, gx, gy, gz, dt_s);
    if (st != SF_OK) return st;
    
    st = SF_UpdateAccel(ctx, ax_g, ay_g, az_g);
    if (st != SF_OK) return st;
    
    return SF_UpdateMag(ctx, mx_g, my_g, mz_g);
}

SF_Status_t SF_GetAttitude(void *ctx, SF_Attitude_t *out)
{
    (void)ctx;
    if (out == 0) {
        return SF_ERROR_INVALID_PARAM;
    }
    out->qw = g_sf.q[0];
    out->qx = g_sf.q[1];
    out->qy = g_sf.q[2];
    out->qz = g_sf.q[3];

    /* Euler extraction from the quaternion, in the standard ZYX
     * convention (yaw-pitch-roll). The pitch singularity at ±90° is
     * inherent to Euler angles and is why the quaternion is the actual
     * state — this is only for display/logging. */
    float qw = g_sf.q[0], qx = g_sf.q[1], qy = g_sf.q[2], qz = g_sf.q[3];

    float sinr_cosp = 2.0f * (qw*qx + qy*qz);
    float cosr_cosp = 1.0f - 2.0f * (qx*qx + qy*qy);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw*qy - qz*qx);
    float pitch;
    if (fabsf(sinp) >= 1.0f) {
        pitch = copysignf((float)M_PI / 2.0f, sinp);
    } else {
        pitch = asinf(sinp);
    }

    float siny_cosp = 2.0f * (qw*qz + qx*qy);
    float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    out->roll_deg  = roll  * DEG_PER_RAD;
    out->pitch_deg = pitch * DEG_PER_RAD;
    out->yaw_deg   = yaw   * DEG_PER_RAD;
    if (out->yaw_deg < 0.0f) {
        out->yaw_deg += 360.0f;
    }

    out->gyro_bias_x = g_sf.bg[0];
    out->gyro_bias_y = g_sf.bg[1];
    out->gyro_bias_z = g_sf.bg[2];

    out->update_count = g_sf.updates_gyro + g_sf.updates_accel;
    out->dt_last_s = g_sf.last_dt_s;
    return SF_OK;
}

/*============================================================================*
 *                          SECOND-STAGE ALTITUDE FILTER                      *
 *============================================================================*/

typedef struct {
    float alt_m;
    float ground_alt_m;
    float vel_m_s;
    float P[2][2];
    float Q[2][2];
    float R_baro;
    float R_gps;
    float baro_bias_m;      /* slowly learned barometric offset */
    float last_dt_s;
    uint32_t baro_updates;
    uint32_t gps_updates;
    int initialized;
} sf_altitude_t;

static sf_altitude_t g_alt;

/** @brief Standard atmosphere pressure altitude [m] from pressure [Pa].
 *         Assumes sea-level reference of 101325 Pa. */
static float sf_pressure_to_altitude(float p_pa)
{
    return 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.190295f));
}

SF_Status_t SF_AltitudeFeedBaro(void *ctx, float pressure_pa, float dt_s)
{
    (void)ctx;
    if (dt_s <= 0.0f || dt_s > SF_MAX_DT_S) {
        return SF_ERROR_INVALID_PARAM;
    }

    float baro_alt = sf_pressure_to_altitude(pressure_pa);

    if (!g_alt.initialized) {
        g_alt.alt_m = baro_alt;
        g_alt.vel_m_s = 0.0f;
        g_alt.P[0][0] = 10.0f;
        g_alt.P[1][1] = 1.0f;
        g_alt.P[0][1] = g_alt.P[1][0] = 0.0f;
        g_alt.R_baro = 1.0f;         /* ~1 m²  barometric measurement noise */
        g_alt.R_gps  = 25.0f;        /* ~25 m² GPS altitude noise (typical) */
        g_alt.Q[0][0] = 0.01f;
        g_alt.Q[1][1] = 0.1f;
        g_alt.baro_bias_m = 0.0f;
        g_alt.last_dt_s = dt_s;
        g_alt.initialized = 1;
        g_alt.baro_updates = 1;
        return SF_OK;
    }

    /* Prediction: constant-velocity model.
     *   alt += vel * dt
     *   vel unchanged
     */
    g_alt.alt_m   += g_alt.vel_m_s * dt_s;

    /* Covariance propagation: P = F P Fᵀ + Q, F = [[1, dt], [0, 1]]. */
    float p00 = g_alt.P[0][0] + 2.0f * dt_s * g_alt.P[0][1] + dt_s * dt_s * g_alt.P[1][1];
    float p01 = g_alt.P[0][1] + dt_s * g_alt.P[1][1];
    float p11 = g_alt.P[1][1];
    g_alt.P[0][0] = p00 + g_alt.Q[0][0];
    g_alt.P[0][1] = g_alt.P[1][0] = p01;
    g_alt.P[1][1] = p11 + g_alt.Q[1][1];

    /* Measurement update: y = baro_alt − (alt + baro_bias). The bias is
     * slowly learned to absorb the weather-scale drift that would
     * otherwise be seen as real motion. */
    float y = baro_alt - (g_alt.alt_m + g_alt.baro_bias_m);

    /* S = H P Hᵀ + R, H = [1, 0]. */
    float S = g_alt.P[0][0] + g_alt.R_baro;
    float K0 = g_alt.P[0][0] / S;
    float K1 = g_alt.P[1][0] / S;

    g_alt.alt_m   += K0 * y;
    g_alt.vel_m_s += K1 * y;

    /* Bias learning: a tiny fraction of the innovation bleeds into the
     * bias, with a time constant of roughly 30 s. Too fast and real
     * climbs get absorbed into the bias; too slow and the barometric
     * drift shows up as phantom altitude change. */
    g_alt.baro_bias_m += 0.003f * y;

    /* Covariance update: P = (I − KH) P. */
    float new_p00 = (1.0f - K0) * g_alt.P[0][0];
    float new_p01 = (1.0f - K0) * g_alt.P[0][1];
    float new_p10 = g_alt.P[1][0] - K1 * g_alt.P[0][0];
    float new_p11 = g_alt.P[1][1] - K1 * g_alt.P[0][1];
    g_alt.P[0][0] = new_p00;
    g_alt.P[0][1] = new_p01;
    g_alt.P[1][0] = new_p10;
    g_alt.P[1][1] = new_p11;

    g_alt.last_dt_s = dt_s;
    g_alt.baro_updates++;
    return SF_OK;
}

SF_Status_t SF_AltitudeFeedGPS(void *ctx, float gps_altitude_m, bool valid)
{
    (void)ctx;
    if (!valid || !g_alt.initialized) {
        return SF_ERROR_INVALID_PARAM;
    }

    /* Measurement update with H = [1, 0]. Same math as the baro update
     * but with a much larger R, so a single GPS altitude sample can
     * only nudge the estimate, not jump it. */
    float y = gps_altitude_m - g_alt.alt_m;
    float S = g_alt.P[0][0] + g_alt.R_gps;
    float K0 = g_alt.P[0][0] / S;
    float K1 = g_alt.P[1][0] / S;

    g_alt.alt_m   += K0 * y;
    g_alt.vel_m_s += K1 * y;

    float new_p00 = (1.0f - K0) * g_alt.P[0][0];
    float new_p01 = (1.0f - K0) * g_alt.P[0][1];
    float new_p10 = g_alt.P[1][0] - K1 * g_alt.P[0][0];
    float new_p11 = g_alt.P[1][1] - K1 * g_alt.P[0][1];
    g_alt.P[0][0] = new_p00;
    g_alt.P[0][1] = new_p01;
    g_alt.P[1][0] = new_p10;
    g_alt.P[1][1] = new_p11;

    g_alt.gps_updates++;
    return SF_OK;
}

void SF_AltitudeSetGroundRef(float pressure_pa)
{
    g_alt.ground_alt_m = sf_pressure_to_altitude(pressure_pa);
}

SF_Status_t SF_GetAltitude(void *ctx, SF_Altitude_t *out)
{
    (void)ctx;
    if (out == 0) {
        return SF_ERROR_INVALID_PARAM;
    }
    out->altitude_m = g_alt.alt_m - g_alt.ground_alt_m;
    out->vertical_velocity_m_s = g_alt.vel_m_s;
    out->pressure_pa = 0.0f;   /* populated by the caller's own BMP388 read */
    out->baro_altitude_m = g_alt.alt_m - g_alt.baro_bias_m - g_alt.ground_alt_m;
    out->gps_altitude_m = 0.0f;
    out->gps_altitude_valid = (g_alt.gps_updates > 0U);
    out->baro_updates = g_alt.baro_updates;
    out->gps_updates = g_alt.gps_updates;
    return SF_OK;
}
