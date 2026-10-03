/**
 ******************************************************************************
 * @file    MPU6050_program.c
 * @brief   Implementation of the InvenSense MPU6050 6-axis IMU driver
 *          declared in MPU6050_interface.h.
 * @details
 *   Pipeline: MPU6050_enumReadData() -> I2C burst read ->
 *             convert with the scale factors chosen at Init() ->
 *             caller-supplied MPU6050_Data_t.
 *
 *   On top of the raw 6-axis data, this driver also offers four derived
 *   quantities computed on the STM32 side:
 *     - MPU6050_enumGetAttitude()  pitch + roll (complementary filter)
 *     - MPU6050_enumGetHeading()   integrated yaw (gyro-only — see warning)
 *     - MPU6050_enumGetSpeed()     dead-reckoned forward speed
 *     - MPU6050_enumGetPosition()  dead-reckoned position, Z axis only
 *
 *   All bus traffic goes through the shared bare-metal I2C protocol
 *   layer in Protocols/I2C/ (i2c_interface.h). All delays go through the
 *   TIM protocol layer (TIM_interface.h). No HAL_* calls anywhere.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#include "MPU6050_interface.h"
#include "MPU6050_private.h"
#include "MPU6050_config.h"
#include "i2c_interface.h"
#include "TIM_interface.h"
#include <math.h>

/*============================================================================*
 *                          INTERNAL CONSTANTS                                *
 *============================================================================*/

/** @brief Pi, for the radian/degree conversions. Defined only if the
 *         toolchain's math.h did not supply it. */
#ifndef M_PI
#define M_PI    3.14159265358979323846f
#endif

/** @brief Milliseconds to wait after the reset command before the part
 *         is guaranteed to be answering again (datasheet §6.1). */
#define MPU6050_RESET_DELAY_MS   100U

/** @brief Number of samples averaged for the start-up zero-offset
 *         calibration. 400 samples at ~2 ms each ≈ 0.8 s, which is a
 *         reasonable still-start window without making boot feel slow. */
#define MPU6050_CALIB_SAMPLES    400U

/** @brief Milliseconds between calibration samples. */
#define MPU6050_CALIB_SAMPLE_MS  2U

/*============================================================================*
 *                          FILE-LOCAL STATE                                  *
 *============================================================================*/

/**
 * @brief Zero-offset per axis, subtracted from every raw reading.
 *
 * @details
 *   A real MPU6050 with the vehicle at rest does not read (0, 0, 1g) —
 *   it reads some small non-zero vector, because of manufacturing
 *   tolerances in the proof-mass suspension and the ADC. Those offsets
 *   integrate without bound in the dead-reckoning functions below, so
 *   they must be removed at the source.
 *
 *   Accel_Offset[2] holds a real offset, not 1 g: the calibration
 *   subtracts the expected 1 g from the Z average before storing, so
 *   the read path can uniformly subtract all three.
 */
static float Accel_Offset[3] = { 0.0f, 0.0f, 0.0f };

/** @brief Gyroscope zero-offset per axis. A gyro reads non-zero while
 *         perfectly still; this cancels that, and without it the
 *         integrated heading/position drift steadily. */
static float Gyro_Offset[3]  = { 0.0f, 0.0f, 0.0f };

/** @brief Complementary-filter state: the fused pitch [degrees]. */
static float Filtered_Pitch = 0.0f;

/** @brief Complementary-filter state: the fused roll [degrees]. */
static float Filtered_Roll = 0.0f;

/** @brief Integrated (relative) heading state [degrees]. See the drift
 *         warning on MPU6050_enumGetHeading(). */
static float Filtered_Heading = 0.0f;

/** @brief Integrated vertical velocity [m/s], used by the Z position
 *         estimate to separate real motion from gravity. */
static float Vert_Velocity_m_s = 0.0f;

/**
 * @brief The WHO_AM_I value read at init — 0x68 on a genuine MPU6050.
 *
 * @note  Non-static so a debugger can see at a glance whether the IMU
 *        answered at all. Volatile because the value is meant to be
 *        inspected live, not because any code races on it.
 */
volatile uint8_t MPU6050_ID = 0U;

/**
 * @brief The I2C configuration this driver talks to the IMU with.
 *
 * @details
 *   Populated from the compile-time defines in MPU6050_config.h. The
 *   protocol-layer I2C driver takes this struct by pointer on every
 *   transaction — it is the "handle" for the bus.
 *
 *   Static const would be ideal, but I2C_enumInit() takes a non-const
 *   pointer (the protocol layer may write back discovered state), so
 *   this stays non-const. The contents are not modified after
 *   declaration.
 */
static I2C_Config_t MPU6050_I2C = {
    .Channel      = MPU6050_I2C_CHANNEL,
    .Mode         = I2C_MODE_I2C,
    .ACK          = I2C_ACK_ENABLE,
    .AddressMode  = I2C_ADDR_MODE_7BIT,
    .OwnAddress   = 0x00U,
    .Speed        = I2C_SPEED_FAST,
    .FM_DutyCycle = I2C_DUTY_2,
    .ClockSpeed_Hz = MPU6050_I2C_CLOCK_HZ,
    .PCLK1_Hz      = MPU6050_I2C_PCLK1_HZ,
};

/*============================================================================*
 *                          INTERNAL BUS WRAPPERS                             *
 *============================================================================*/

/**
 * @brief Write one MPU6050 register.
 *
 * @details
 *   Single I2C transaction: START, address+W, [reg, data], STOP.
 *
 * @param[in] reg   Register address.
 * @param[in] data  Byte to write.
 */
static void MPU6050_vWriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    (void)I2C_enumMasterTransmit(&MPU6050_I2C, MPU6050_I2C_ADDR, buf, 2U);
}

/**
 * @brief Read one MPU6050 register.
 *
 * @details
 *   Write-then-read with a repeated START:
 *     START, addr+W, [reg], repeated START, addr+R, [value], STOP
 *
 *   The failure mode of this particular wrapper is deliberately quiet
 *   (returns 0 on a bus failure) — it is used for one-off reads during
 *   Init(), where an outright NACK will be caught a moment later by the
 *   WHO_AM_I check anyway. The burst path below is the one that reports
 *   errors, because that's the one called in the read loop.
 *
 * @param[in] reg  Register address.
 *
 * @return The register's value, or 0 on a bus failure.
 */
static uint8_t MPU6050_u8ReadReg(uint8_t reg)
{
    uint8_t val = 0U;
    (void)I2C_enumMasterTransmitReceive(&MPU6050_I2C, MPU6050_I2C_ADDR,
                                        &reg, 1U, &val, 1U);
    return val;
}

/**
 * @brief Burst-read a contiguous block of MPU6050 registers in one
 *        transaction.
 *
 * @details
 *   The MPU6050 auto-increments its register pointer on multi-byte
 *   reads, so a single START/address-write/reg/repeated-START/read-N
 *   gets the whole block from one coherent sample.
 *
 * @param[in]  startReg  First register address.
 * @param[out] buf       Receives @p len bytes.
 * @param[in]  len       Number of bytes to read.
 *
 * @return OK on success, or the I2C driver's error code on a bus
 *         failure / NACK.
 */
static ErrorState_t MPU6050_enumReadBurst(uint8_t startReg, uint8_t *buf, uint16_t len)
{
    return I2C_enumMasterTransmitReceive(&MPU6050_I2C, MPU6050_I2C_ADDR,
                                         &startReg, 1U, buf, len);
}

/*============================================================================*
 *                          PUBLIC API IMPLEMENTATION                         *
 *============================================================================*/

ErrorState_t MPU6050_enumInit(void)
{
    /* 1. Bring up the I2C peripheral. */
    if (I2C_enumInit(&MPU6050_I2C) != OK) {
        return NOK;
    }

    /* 2. Reset the device. The reset also restores the default ±2 g /
     *    ±250 dps ranges, so the range writes below are from a known
     *    starting point regardless of any previous configuration. */
    MPU6050_vWriteReg(PWR_MGMT_1, PWR_RESET);
    TIM_vDelayMs(MPU6050_DELAY_TIMER, MPU6050_RESET_DELAY_MS);

    /* 3. Wake up with the gyro-referenced PLL as clock source. */
    MPU6050_vWriteReg(PWR_MGMT_1, CLOCK_SEL_PLL);
    TIM_vDelayMs(MPU6050_DELAY_TIMER, 50U);

    /* 4. Configure the low-pass filter (~44 Hz accel / ~42 Hz gyro —
     *    the same single shared DLPF field the MPU6050 exposes). */
    MPU6050_vWriteReg(CONFIG, 0x03U);

    /* 5. Full-scale ranges, from the compile-time config. */
    MPU6050_vWriteReg(GYRO_CONFIG,  (uint8_t)(MPU6050_GYRO_FS  << 3U));
    MPU6050_vWriteReg(ACCEL_CONFIG, (uint8_t)(MPU6050_ACCEL_FS << 3U));

    /* 6. Identity check. A mis-wired or absent part shows up here, not
     *    later as plausible-looking garbage. */
    MPU6050_ID = MPU6050_u8ReadReg(WHO_AM_I);
    if (MPU6050_ID != 0x68U) {
        return NOK;
    }

    /* 7. Still-start calibration: average a window of raw samples and
     *    store the mean as the per-axis zero offset. Accel Z is
     *    special-cased — the expected value is 1 g, not 0, so the
     *    offset stored is (mean − 1 g), which lets the read path
     *    uniformly subtract Accel_Offset[i] from all three axes. */
    float sum[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    for (uint32_t i = 0U; i < MPU6050_CALIB_SAMPLES; i++) {
        uint8_t b[14];
        if (MPU6050_enumReadBurst(ACCEL_XOUT_H, b, 14U) == OK) {
            sum[0] += (float)(int16_t)(((uint16_t)b[0]  << 8U) | (uint16_t)b[1]);
            sum[1] += (float)(int16_t)(((uint16_t)b[2]  << 8U) | (uint16_t)b[3]);
            sum[2] += (float)(int16_t)(((uint16_t)b[4]  << 8U) | (uint16_t)b[5]);
            sum[3] += (float)(int16_t)(((uint16_t)b[8]  << 8U) | (uint16_t)b[9]);
            sum[4] += (float)(int16_t)(((uint16_t)b[10] << 8U) | (uint16_t)b[11]);
            sum[5] += (float)(int16_t)(((uint16_t)b[12] << 8U) | (uint16_t)b[13]);
        }
        TIM_vDelayMs(MPU6050_DELAY_TIMER, MPU6050_CALIB_SAMPLE_MS);
    }

    Accel_Offset[0] = sum[0] / (float)MPU6050_CALIB_SAMPLES;
    Accel_Offset[1] = sum[1] / (float)MPU6050_CALIB_SAMPLES;
    Accel_Offset[2] = (sum[2] / (float)MPU6050_CALIB_SAMPLES) - ACCEL_SENS_2G;

    Gyro_Offset[0]  = sum[3] / (float)MPU6050_CALIB_SAMPLES;
    Gyro_Offset[1]  = sum[4] / (float)MPU6050_CALIB_SAMPLES;
    Gyro_Offset[2]  = sum[5] / (float)MPU6050_CALIB_SAMPLES;

    return OK;
}

ErrorState_t MPU6050_enumReadData(MPU6050_Data_t *Copy_pData)
{
    if (Copy_pData == 0) {
        return NULL_POINTER;
    }

    uint8_t buffer[14];

    /* One contiguous burst from ACCEL_XOUT_H through GYRO_ZOUT_L:
     * accel, temp and gyro all come from the same sample, which is what
     * keeps the axes coherent — separate reads could straddle two
     * samples and describe a vehicle orientation that never existed. */
    if (MPU6050_enumReadBurst(ACCEL_XOUT_H, buffer, 14U) != OK) {
        return NOK;
    }

    Copy_pData->AccelX = ((float)(int16_t)(((uint16_t)buffer[0] << 8U) | (uint16_t)buffer[1]) - Accel_Offset[0]) / ACCEL_SENS_2G;
    Copy_pData->AccelY = ((float)(int16_t)(((uint16_t)buffer[2] << 8U) | (uint16_t)buffer[3]) - Accel_Offset[1]) / ACCEL_SENS_2G;
    Copy_pData->AccelZ = ((float)(int16_t)(((uint16_t)buffer[4] << 8U) | (uint16_t)buffer[5]) - Accel_Offset[2]) / ACCEL_SENS_2G;

    /* Temperature conversion per the MPU-6050 register map datasheet. */
    Copy_pData->Temperature = ((float)(int16_t)(((uint16_t)buffer[6] << 8U) | (uint16_t)buffer[7])
                               / TEMP_SENSITIVITY) + TEMP_OFFSET_C;

    Copy_pData->GyroX = ((float)(int16_t)(((uint16_t)buffer[8]  << 8U) | (uint16_t)buffer[9])  - Gyro_Offset[0]) / GYRO_SENS_250;
    Copy_pData->GyroY = ((float)(int16_t)(((uint16_t)buffer[10] << 8U) | (uint16_t)buffer[11]) - Gyro_Offset[1]) / GYRO_SENS_250;
    Copy_pData->GyroZ = ((float)(int16_t)(((uint16_t)buffer[12] << 8U) | (uint16_t)buffer[13]) - Gyro_Offset[2]) / GYRO_SENS_250;

    return OK;
}

ErrorState_t MPU6050_enumGetAttitude(MPU6050_Data_t *Copy_pData,
                                     float Copy_fDt,
                                     float *Copy_pfPitch,
                                     float *Copy_pfRoll)
{
    if (Copy_pData == 0 || Copy_pfPitch == 0 || Copy_pfRoll == 0) {
        return NULL_POINTER;
    }

    /* Accelerometer-derived angles. atan2f handles the full circle and
     * avoids the division-by-zero blowup of atan(ay/az). */
    float accP = atan2f(-Copy_pData->AccelX,
                        sqrtf(Copy_pData->AccelY * Copy_pData->AccelY
                            + Copy_pData->AccelZ * Copy_pData->AccelZ))
                 * (180.0f / M_PI);
    float accR = atan2f(Copy_pData->AccelY, Copy_pData->AccelZ)
                 * (180.0f / M_PI);

    /* Complementary filter: 95 % gyro (smooth, drifts), 5 % accel
     * (steady, noisy). Fast response chosen over a slower blend because
     * the Z-tracking consumer needs to follow quick attitude changes. */
    Filtered_Pitch = 0.95f * (Filtered_Pitch + Copy_pData->GyroY * Copy_fDt) + 0.05f * accP;
    Filtered_Roll  = 0.95f * (Filtered_Roll  + Copy_pData->GyroX * Copy_fDt) + 0.05f * accR;

    *Copy_pfPitch = Filtered_Pitch;
    *Copy_pfRoll  = Filtered_Roll;
    return OK;
}

ErrorState_t MPU6050_enumGetHeading(MPU6050_Data_t *Copy_pData,
                                    float Copy_fDt,
                                    float *Copy_pfHeading)
{
    if (Copy_pData == 0 || Copy_pfHeading == 0) {
        return NULL_POINTER;
    }

    /* No magnetometer on this part: pure integration of GyroZ. Relative
     * and drift-prone by nature — see the warning on the interface
     * declaration. The sign is a config choice because the sensor's
     * Z-axis convention depends on how the board is mounted. */
    Filtered_Heading += MPU6050_GYROZ_SIGN * Copy_pData->GyroZ * Copy_fDt;

    /* Wrap into [0, 360). */
    if (Filtered_Heading >= 360.0f) {
        Filtered_Heading -= 360.0f;
    }
    if (Filtered_Heading < 0.0f) {
        Filtered_Heading += 360.0f;
    }

    *Copy_pfHeading = Filtered_Heading;
    return OK;
}

ErrorState_t MPU6050_enumGetSpeed(MPU6050_Data_t *Copy_pData,
                                  float Copy_fDt,
                                  float *Copy_pfSpeed)
{
    if (Copy_pData == 0 || Copy_pfSpeed == 0) {
        return NULL_POINTER;
    }

    float total_acc = sqrtf(Copy_pData->AccelX * Copy_pData->AccelX
                          + Copy_pData->AccelY * Copy_pData->AccelY
                          + Copy_pData->AccelZ * Copy_pData->AccelZ);

    /* Convert g to m/s² (1 g = 9.80665 m/s²). The total_acc - 1.0f term
     * removes the ~1 g that gravity contributes even at rest. */
    float linear_acc_m_s2 = (total_acc - 1.0f) * 9.80665f;

    /* 1. Stillness detection (ZUPT). If the gyro and the residual
     *    linear acceleration are both below their noise floors, we
     *    assume the vehicle is stationary and force the speed toward
     *    zero. Without this, any residual bias integrates into a
     *    fictitious creep that never goes away. */
    if (fabsf(Copy_pData->GyroX) < 1.5f
     && fabsf(Copy_pData->GyroY) < 1.5f
     && fabsf(Copy_pData->GyroZ) < 1.5f) {
        if (fabsf(linear_acc_m_s2) < 0.35f) {
            *Copy_pfSpeed *= 0.70f;
            if (fabsf(*Copy_pfSpeed) < 0.05f) {
                *Copy_pfSpeed = 0.0f;
            }
            linear_acc_m_s2 = 0.0f;
        }
    }

    /* 2. Deadzone: ignore anything below the accelerometer's own noise
     *    floor so vibration doesn't accumulate into drift. */
    if (fabsf(linear_acc_m_s2) < 0.20f) {
        linear_acc_m_s2 = 0.0f;
    }

    /* 3. Trapezoidal integration. */
    *Copy_pfSpeed += linear_acc_m_s2 * Copy_fDt;

    /* 4. Physical floor: this integration model doesn't represent
     *    reverse motion, so clamp negatives and snap tiny values to
     *    zero instead of letting them dribble. */
    if (*Copy_pfSpeed < 0.0f) {
        *Copy_pfSpeed = 0.0f;
    }
    if (*Copy_pfSpeed < 0.015f) {
        *Copy_pfSpeed = 0.0f;
    }

    return OK;
}

ErrorState_t MPU6050_enumGetPosition(MPU6050_Data_t *Copy_pData,
                                     float Copy_fSpeed,
                                     float Copy_fHeading,
                                     float Copy_fPitch,
                                     float Copy_fDt,
                                     MPU6050_Position_t *Copy_pPos)
{
    if (Copy_pData == 0 || Copy_pPos == 0) {
        return NULL_POINTER;
    }
    (void)Copy_fSpeed;    /* kept in the signature for a future XY step — see below */
    (void)Copy_fHeading;

    float radP = Filtered_Pitch * (M_PI / 180.0f);
    float radR = Filtered_Roll  * (M_PI / 180.0f);

    /* Gravity-compensated world-frame acceleration along Z. Rotation
     * from body to world uses the current pitch and roll; the −1 g term
     * removes the gravity component that survives the rotation (the
     * pitch/roll estimates are themselves derived from that same
     * gravity vector, so at rest the result is exactly zero). */
    float az_world = (Copy_pData->AccelZ * cosf(radP) * cosf(radR))
                   - (Copy_pData->AccelX * sinf(radP))
                   + (Copy_pData->AccelY * sinf(radR) * cosf(radP))
                   - 1.0f;
    float az_m_s2 = az_world * 9.80665f;

    /* Smart vertical stillness detection: same idea as the speed path,
     * but tuned for the vertical channel and slightly looser thresholds
     * because the Z axis has more gravitational coupling. */
    float total_gyro = fabsf(Copy_pData->GyroX)
                     + fabsf(Copy_pData->GyroY)
                     + fabsf(Copy_pData->GyroZ);
    if (total_gyro < 1.5f && fabsf(az_m_s2) < 0.35f) {
        az_m_s2 = 0.0f;
        Vert_Velocity_m_s *= 0.75f;   /* snap velocity toward zero when stationary */
    }

    /* Leaky integration: only integrate meaningful acceleration, and
     * bleed velocity a little on every step. Together these two rules
     * keep the estimate stable at rest — without the bleed, even a
     * tiny residual bias would let the velocity wander. */
    if (fabsf(az_m_s2) > 0.20f) {
        Vert_Velocity_m_s += az_m_s2 * Copy_fDt;
    }
    Vert_Velocity_m_s *= 0.97f;

    Copy_pPos->Z += Vert_Velocity_m_s * Copy_fDt;

    /* When stationary, bleed Z back toward its reference (0). This is
     * what stops the dead-reckoned altitude from accumulating the
     * sensor's residual bias into a slow, one-way crawl. At the
     * expected 50 ms call interval the time constant is roughly a few
     * seconds. */
    if (total_gyro < 1.5f && fabsf(Vert_Velocity_m_s) < 0.01f) {
        Copy_pPos->Z *= 0.98f;
        if (fabsf(Copy_pPos->Z) < 0.005f) {
            Copy_pPos->Z = 0.0f;   /* snap at 5 mm to kill last-bit jitter */
        }
    }

    /* X and Y are left at whatever the caller had. The driver has no
     * horizontal position reference — a real XY estimate needs a fused
     * heading + speed, which belongs in the sensor-fusion layer, not
     * here. The signature keeps the parameters so the future fusion
     * code has a stable call shape to target. */
    Copy_pPos->X = 0.0f;
    Copy_pPos->Y = 0.0f;

    return OK;
}
