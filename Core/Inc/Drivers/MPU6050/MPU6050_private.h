/**
 ******************************************************************************
 * @file    MPU6050_private.h
 * @brief   MPU6050 register map, bit values and scale factors —
 *          driver-internal.
 * @details
 *   Register addresses from the InvenSense MPU-6050 register map
 *   (RM-MPU-6000A-00). Nothing here is part of the driver's public API;
 *   include MPU6050_interface.h instead.
 *
 * @note  The MPU6050 is a 6-axis part (accelerometer + gyroscope) in a
 *        single die — unlike the MPU9250, it has **no on-board
 *        magnetometer**, so there is no AK8963 and no need to drive an
 *        internal I2C master to reach one. Every register below is read
 *        or written directly over the STM32's own I2C bus.
 *
 * @note  Multi-byte sensor values are big-endian: the `_H` register
 *        holds the high byte and `_L` the low byte, so a reading is
 *        `(H << 8) | L`.
 ******************************************************************************
 */

#ifndef MPU6050_PRIVATE_H
#define MPU6050_PRIVATE_H

/*============================================================================*
 *                          SELF-TEST REGISTERS                               *
 *============================================================================*/

/**
 * @name Self-test registers
 *
 * Factory self-test values, used to check the part against its trim
 * values. The driver does not run a self-test, so these are declared but
 * unused.
 *
 * @note  Addresses differ from the MPU9250 / 6500: on the MPU6050 all
 *        four self-test bytes are contiguous, with the accelerometer's
 *        Z-axis trim bits split across SELF_TEST_A alongside X/Y/Z.
 * @{
 */
#define SELF_TEST_X   0x0DU   /**< Accelerometer/gyroscope X-axis self-test bits. */
#define SELF_TEST_Y   0x0EU   /**< Accelerometer/gyroscope Y-axis self-test bits. */
#define SELF_TEST_Z   0x0FU   /**< Accelerometer/gyroscope Z-axis self-test bits. */
#define SELF_TEST_A   0x10U   /**< Accelerometer self-test bits, low bits of
                                   all 3 axes. */
/** @} */

/*============================================================================*
 *                        CONFIGURATION REGISTERS                             *
 *============================================================================*/

/**
 * @name Configuration registers
 * @{
 */
#define SMPLRT_DIV    0x19U   /**< Sample-rate divider: output rate =
                                   internal rate / (1 + this). */
#define CONFIG        0x1AU   /**< Low-pass filter bandwidth (DLPF_CFG) and
                                   FIFO overflow behaviour.
                                   @note Unlike the MPU6500/9250, the
                                   MPU6050 has no separate accelerometer
                                   LPF register — this one DLPF_CFG field
                                   filters *both* the accelerometer and
                                   the gyroscope. */
#define GYRO_CONFIG   0x1BU   /**< Gyroscope full-scale range
                                   (±250/500/1000/2000 °/s) and self-test
                                   enables. */
#define ACCEL_CONFIG  0x1CU   /**< Accelerometer full-scale range
                                   (±2/4/8/16 g) and self-test enables. */
#define FIFO_EN       0x23U   /**< Which sensors are written into the FIFO
                                   (the driver polls instead, so this stays
                                   0). */
/** @} */

/*============================================================================*
 *                          INTERRUPT REGISTERS                               *
 *============================================================================*/

/**
 * @name Interrupt registers
 * @{
 */
#define INT_PIN_CFG   0x37U   /**< INT pin behaviour: polarity, latching, and
                                   the I2C bypass enable. */
#define INT_ENABLE    0x38U   /**< Which conditions raise the INT pin
                                   (data-ready, FIFO overflow). */
#define INT_STATUS    0x3AU   /**< Which condition actually fired; reading it
                                   clears the flags. */
/** @} */

/*============================================================================*
 *                          SENSOR DATA REGISTERS                             *
 *============================================================================*/

/**
 * @name Sensor data registers
 *
 * Big-endian: a reading is `(H << 8) | L`, and the whole block must be
 * read as one burst so every axis (and the temperature) comes from the
 * same sample.
 * @{
 */
#define ACCEL_XOUT_H  0x3BU   /**< Accelerometer X, high byte. */
#define ACCEL_XOUT_L  0x3CU   /**< Accelerometer X, low byte. */
#define ACCEL_YOUT_H  0x3DU   /**< Accelerometer Y, high byte. */
#define ACCEL_YOUT_L  0x3EU   /**< Accelerometer Y, low byte. */
#define ACCEL_ZOUT_H  0x3FU   /**< Accelerometer Z, high byte. */
#define ACCEL_ZOUT_L  0x40U   /**< Accelerometer Z, low byte. */
#define TEMP_OUT_H    0x41U   /**< Die temperature, high byte. */
#define TEMP_OUT_L    0x42U   /**< Die temperature, low byte. */
#define GYRO_XOUT_H   0x43U   /**< Gyroscope X, high byte. */
#define GYRO_XOUT_L   0x44U   /**< Gyroscope X, low byte. */
#define GYRO_YOUT_H   0x45U   /**< Gyroscope Y, high byte. */
#define GYRO_YOUT_L   0x46U   /**< Gyroscope Y, low byte. */
#define GYRO_ZOUT_H   0x47U   /**< Gyroscope Z, high byte. */
#define GYRO_ZOUT_L   0x48U   /**< Gyroscope Z, low byte. */
/** @} */

/*============================================================================*
 *                        POWER AND IDENTITY REGISTERS                        *
 *============================================================================*/

/**
 * @name Power and identity registers
 * @{
 */
#define USER_CTRL     0x6AU   /**< Enables the FIFO and the internal I2C
                                   master (unused here — there's no
                                   auxiliary sensor to fetch, so the
                                   driver leaves this at reset). */
#define PWR_MGMT_1    0x6BU   /**< Power management 1: device reset, sleep,
                                   and the clock source. */
#define PWR_MGMT_2    0x6CU   /**< Power management 2: per-axis standby for
                                   the accelerometer and gyroscope. */
#define WHO_AM_I      0x75U   /**< Device ID; reads back 0x68 on a genuine
                                   MPU6050 (0x71 is the MPU9250/6500). */
/** @} */

/*============================================================================*
 *                          REGISTER BIT VALUES                               *
 *============================================================================*/

/**
 * @name Register bit values
 * @{
 */
#define PWR_RESET       0x80U   /**< PWR_MGMT_1 — trigger a full device
                                     reset. */
#define CLOCK_SEL_PLL   0x01U   /**< PWR_MGMT_1 — use the gyro-referenced PLL,
                                     which is far more stable than the
                                     internal oscillator. */
#define I2C_IF_DIS      0x10U   /**< USER_CTRL — disable the part's I2C slave
                                     interface and force SPI-only mode
                                     (the MPU6050 supports SPI on some
                                     breakouts). Must stay CLEAR on this
                                     board: the driver talks to the part
                                     over I2C, and setting this bit would
                                     cut that link off. Kept here only for
                                     reference/completeness. */
/** @} */

/*============================================================================*
 *                              SCALE FACTORS                                 *
 *============================================================================*/

/**
 * @name Scale factors
 *
 * Raw counts to physical units, for the ranges this driver configures.
 * @{
 */
#define ACCEL_SENS_2G   16384.0f   /**< LSB per g at the ±2 g range:
                                        acceleration [g] = raw / this. */
#define GYRO_SENS_250   131.0f     /**< LSB per °/s at the ±250 °/s range:
                                        rate [°/s] = raw / this. */
/** @} */

/*============================================================================*
 *                          TEMPERATURE CONVERSION                            *
 *============================================================================*/

/**
 * @name Temperature conversion
 *
 * Per the MPU-6050 register map datasheet:
 * Temperature [°C] = raw / TEMP_SENSITIVITY + TEMP_OFFSET_C.
 * (The MPU9250 uses a different pair of constants — 333.87 and 21 — so
 * this is not portable between the two parts.)
 * @{
 */
#define TEMP_SENSITIVITY  340.0f   /**< LSB per °C. */
#define TEMP_OFFSET_C     36.53f   /**< °C at raw = 0. */
/** @} */

#endif /* MPU6050_PRIVATE_H */
