/**
 * @file MPU6050_program.c
 * @author Alaa Hassan
 * @brief Professional MPU6050 driver (I2C, STM32F411CEU6): High-Accuracy Speed (m/s)
 *        & Gravity-Compensated Altitude
 */
#include "../Inc/Drivers/MPU6050/MPU6050_interface.h"
#include "../Inc/Drivers/MPU6050/MPU6050_private.h"
#include "../Inc/Drivers/MPU6050/MPU6050_config.h"
#include "../Inc/Protocols/I2C/i2c_interface.h"
#include "../../Drivers/TIM/TIM_interface.h"
#include <math.h>

/** @brief Pi, for the radian/degree conversions. Defined only if the toolchain's `math.h` did not. */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Global State */
/** @brief Accelerometer zero-offset per axis, subtracted from every raw reading. */
static float Accel_Offset[3] = { 0, 0, 0 };
/** @brief Gyroscope zero-offset per axis. A gyro reads non-zero while perfectly still; this cancels that, and without it the integrated heading/position drift steadily. */
static float Gyro_Offset[3] = { 0, 0, 0 };
/** @brief Complementary-filter state: the fused pitch [degrees]. */
static float Filtered_Pitch = 0;
/** @brief Complementary-filter state: the fused roll [degrees]. */
static float Filtered_Roll = 0;
/** @brief Integrated (relative) heading state [degrees] — see the drift warning on @ref MPU6050_enumGetHeading. */
static float Filtered_Heading = 0;
/** @brief Integrated vertical velocity [m/s], used to separate real motion from gravity. */
static float Vert_Velocity_m_s = 0;
/** @brief The `WHO_AM_I` value read at init — 0x68 on a genuine part. Kept as a global so a debugger can see at a glance whether the IMU answered at all. */
volatile uint8_t MPU6050_ID = 0;

/** @brief The I2C settings this driver talks to the IMU with. */
static I2C_Config_t MPU6050_I2C = {
  .Channel = MPU6050_I2C_CHANNEL,
  .Mode = I2C_MODE_I2C,
  .ACK = I2C_ACK_ENABLE,
  .AddressMode = I2C_ADDR_MODE_7BIT,
  .OwnAddress = 0x00,
  .Speed = I2C_SPEED_FAST,
  .FM_DutyCycle = I2C_DUTY_2,
  .ClockSpeed_Hz = MPU6050_I2C_CLOCK_HZ,
  .PCLK1_Hz = MPU6050_I2C_PCLK1_HZ,
};

/* Helpers */

/**
 * @brief Write one MPU6050 register.
 * @param reg  Register address.
 * @param data Byte to write.
 * @note Single I2C transaction: START, address+W, [reg, data], STOP.
 */
static void MPU6050_vWriteReg(uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = { reg, data };
  I2C_enumMasterTransmit(&MPU6050_I2C, MPU6050_I2C_ADDR, buf, 2);
}

/**
 * @brief Read one MPU6050 register.
 * @param reg Register address.
 * @return The register's value.
 * @note Write-then-read with a repeated START: address+W [reg], repeated
 *       START, address+R, [value].
 */
static uint8_t MPU6050_u8ReadReg(uint8_t reg)
{
  uint8_t val = 0;
  I2C_enumMasterTransmitReceive(&MPU6050_I2C, MPU6050_I2C_ADDR, &reg, 1, &val, 1);
  return val;
}

/**
 * @brief Burst-read a contiguous block of MPU6050 registers in one transaction.
 * @param startReg First register address; the part auto-increments from here.
 * @param[out] buf  Receives @p len bytes.
 * @param len       Number of bytes to read.
 * @return OK on success, or the I2C driver's error code on a bus failure/NACK.
 */
static ErrorState_t MPU6050_enumReadBurst(uint8_t startReg, uint8_t *buf, uint16_t len)
{
  return I2C_enumMasterTransmitReceive(&MPU6050_I2C, MPU6050_I2C_ADDR, &startReg, 1, buf, len);
}

ErrorState_t MPU6050_enumInit(void)
{
  if (I2C_enumInit(&MPU6050_I2C) != OK)
    return NOK;

  MPU6050_vWriteReg(PWR_MGMT_1, PWR_RESET);
  TIM_vDelayMs(MPU6050_DELAY_TIMER, 150);
  MPU6050_vWriteReg(PWR_MGMT_1, CLOCK_SEL_PLL);
  TIM_vDelayMs(MPU6050_DELAY_TIMER, 50);
  MPU6050_vWriteReg(CONFIG, 0x03);        /* DLPF: ~44 Hz accel / ~42 Hz gyro, shared filter on this part */
  MPU6050_vWriteReg(GYRO_CONFIG, (uint8_t)(MPU6050_GYRO_FS << 3));
  MPU6050_vWriteReg(ACCEL_CONFIG, (uint8_t)(MPU6050_ACCEL_FS << 3));

  MPU6050_ID = MPU6050_u8ReadReg(WHO_AM_I);
  if (MPU6050_ID != 0x68)
    return NOK; /* wrong/absent part: check wiring and MPU6050_I2C_ADDR */

  float sum[6] = { 0 };
  for (int i = 0; i < 400; i++)
  {
    uint8_t b[14];
    if (MPU6050_enumReadBurst(ACCEL_XOUT_H, b, 14) == OK)
    {
      sum[0] += (int16_t)((b[0] << 8) | b[1]);
      sum[1] += (int16_t)((b[2] << 8) | b[3]);
      sum[2] += (int16_t)((b[4] << 8) | b[5]);
      sum[3] += (int16_t)((b[8] << 8) | b[9]);
      sum[4] += (int16_t)((b[10] << 8) | b[11]);
      sum[5] += (int16_t)((b[12] << 8) | b[13]);
    }
    TIM_vDelayMs(MPU6050_DELAY_TIMER, 2);
  }
  for (int i = 0; i < 2; i++)
    Accel_Offset[i] = sum[i] / 400.0f;
  Accel_Offset[2] = (sum[2] / 400.0f) - ACCEL_SENS_2G;
  for (int i = 0; i < 3; i++)
    Gyro_Offset[i] = sum[i + 3] / 400.0f;

  return OK;
}

ErrorState_t MPU6050_enumReadData(MPU6050_Data_t *D)
{
  if (!D)
    return NULL_POINTER;
  uint8_t buffer[14];
  /* One contiguous burst from ACCEL_XOUT_H through GYRO_ZOUT_L: accel, temp
   * and gyro all come from the same sample, which is what keeps the axes
   * coherent — separate reads could straddle two samples. */
  if (MPU6050_enumReadBurst(ACCEL_XOUT_H, buffer, 14) != OK)
    return NOK;
  D->AccelX = ((float)((int16_t)((buffer[0] << 8) | buffer[1])) - Accel_Offset[0]) / ACCEL_SENS_2G;
  D->AccelY = ((float)((int16_t)((buffer[2] << 8) | buffer[3])) - Accel_Offset[1]) / ACCEL_SENS_2G;
  D->AccelZ = ((float)((int16_t)((buffer[4] << 8) | buffer[5])) - Accel_Offset[2]) / ACCEL_SENS_2G;
  D->Temperature = ((float)((int16_t)((buffer[6] << 8) | buffer[7])) / TEMP_SENSITIVITY) + TEMP_OFFSET_C;
  D->GyroX = ((float)((int16_t)((buffer[8] << 8) | buffer[9])) - Gyro_Offset[0]) / GYRO_SENS_250;
  D->GyroY = ((float)((int16_t)((buffer[10] << 8) | buffer[11])) - Gyro_Offset[1]) / GYRO_SENS_250;
  D->GyroZ = ((float)((int16_t)((buffer[12] << 8) | buffer[13])) - Gyro_Offset[2]) / GYRO_SENS_250;
  return OK;
}

ErrorState_t MPU6050_enumGetAttitude(MPU6050_Data_t *D, float dt, float *P, float *R)
{
  if (!D || !P || !R)
    return NULL_POINTER;
  float accP = atan2f(-D->AccelX, sqrtf(D->AccelY * D->AccelY + D->AccelZ * D->AccelZ)) * (180.0f / M_PI);
  float accR = atan2f(D->AccelY, D->AccelZ) * (180.0f / M_PI);

  /* Fast Response Filter (0.95) for better Z tracking */
  Filtered_Pitch = 0.95f * (Filtered_Pitch + D->GyroY * dt) + 0.05f * accP;
  Filtered_Roll = 0.95f * (Filtered_Roll + D->GyroX * dt) + 0.05f * accR;

  *P = Filtered_Pitch;
  *R = Filtered_Roll;
  return OK;
}

ErrorState_t MPU6050_enumGetHeading(MPU6050_Data_t *D, float dt, float *h)
{
  if (!D || !h)
    return NULL_POINTER;

  /* No magnetometer on this part: pure integration of GyroZ. Relative and
   * drift-prone by nature — see the warning on the interface declaration. */
  Filtered_Heading += MPU6050_GYROZ_SIGN * D->GyroZ * dt;

  if (Filtered_Heading >= 360.0f)
    Filtered_Heading -= 360.0f;
  if (Filtered_Heading < 0.0f)
    Filtered_Heading += 360.0f;

  *h = Filtered_Heading;
  return OK;
}

ErrorState_t MPU6050_enumGetSpeed(MPU6050_Data_t *D, float dt, float *s)
{
  if (!D || !s)
    return NULL_POINTER;

  float total_acc = sqrtf(D->AccelX * D->AccelX + D->AccelY * D->AccelY + D->AccelZ * D->AccelZ);
  /* Convert G to m/s^2 (1g = 9.80665 m/s^2) */
  float linear_acc_m_s2 = (total_acc - 1.0f) * 9.80665f;

  /* 1. Enhanced ZUPT Logic (Stillness Detection) */
  /* If Gyro noise and Accel variation are below thresholds, force deceleration */
  if (fabs(D->GyroX) < 1.5f && fabs(D->GyroY) < 1.5f && fabs(D->GyroZ) < 1.5f)
  {
    if (fabs(linear_acc_m_s2) < 0.35f)
    {
      /* Apply strong damping to bleed off drift speed */
      *s *= 0.70f;
      if (fabs(*s) < 0.05f)
        *s = 0.0f;
      linear_acc_m_s2 = 0.0f;
    }
  }

  /* 2. Deadzone to ignore sensor floor noise */
  if (fabs(linear_acc_m_s2) < 0.20f)
  {
    linear_acc_m_s2 = 0.0f;
  }

  /* 3. Trapezoidal Integration */
  *s += linear_acc_m_s2 * dt;

  /* 4. Physical Limit: Speed cannot be negative in this integration model */
  if (*s < 0)
    *s = 0;
  if (*s < 0.015f)
    *s = 0; /* 1.5 cm/s clamp in meters */

  return OK;
}

ErrorState_t MPU6050_enumGetPosition(MPU6050_Data_t *D, float s, float h, float pit, float dt, MPU6050_Position_t *pos)
{
  if (!D || !pos)
    return NULL_POINTER;

  float radP = Filtered_Pitch * (M_PI / 180.0f);
  float radR = Filtered_Roll * (M_PI / 180.0f);

  /* Gravity-Compensated World Acceleration in m/s^2 */
  float az_world = (D->AccelZ * cosf(radP) * cosf(radR)) - (D->AccelX * sinf(radP)) + (D->AccelY * sinf(radR) * cosf(radP)) - 1.0f;
  float az_m_s2 = az_world * 9.80665f;

  /* Smart Vertical Stillness Detection */
  float total_gyro = fabs(D->GyroX) + fabs(D->GyroY) + fabs(D->GyroZ);
  if (total_gyro < 1.5f && fabs(az_m_s2) < 0.35f)
  {
    az_m_s2 = 0;
    Vert_Velocity_m_s *= 0.75f; /* Snap speed to zero when stationary */
  }

  /* Leaky Integration to fix the Negative Drifting Issue */
  if (fabs(az_m_s2) > 0.20f)
  {
    Vert_Velocity_m_s += az_m_s2 * dt;
  }
  Vert_Velocity_m_s *= 0.97f; /* Continuous bleed to stabilize at rest */

  /* Update Distance Z in Meters */
  pos->Z += Vert_Velocity_m_s * dt;

  /* When stationary: bleed Z back toward 0 (reference point)
   * Avoids drift accumulation — at 50ms cycles: ~5s to converge back to 0 */
  if (total_gyro < 1.5f && fabs(Vert_Velocity_m_s) < 0.01f)
  {
    pos->Z *= 0.98f;
    if (fabs(pos->Z) < 0.005f)
      pos->Z = 0.0f; /* snap at 5mm */
  }

  pos->X = 0;
  pos->Y = 0;
  return OK;
}
