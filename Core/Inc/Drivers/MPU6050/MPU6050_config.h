/**
 ******************************************************************************
 * @file    MPU6050_config.h
 * @brief   Compile-time tuning for the MPU6050 driver: wiring, ranges,
 *          and the sign of the yaw-integration term.
 * @details
 *   Everything in this file is a compile-time decision — nothing here
 *   is settable at runtime. The idea is that the two things that vary
 *   board-to-board (which I2C peripheral the IMU hangs off, and which
 *   AD0-strapped address it answers to) live in one place, next to the
 *   two things that vary application-to-application (which full-scale
 *   range you want, and which way round the yaw integration runs).
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef MPU6050_CONFIG_H
#define MPU6050_CONFIG_H

/**
 * @addtogroup hal_mpu6050
 * @{
 */

/*============================================================================*
 *                                  WIRING                                    *
 *============================================================================*/

/**
 * @brief The I2C peripheral the MPU6050 hangs off.
 *
 * @note  Must be one of the I2C_Channel_t values from
 *        Protocols/I2C/i2c_interface.h (I2C_CHANNEL1 / 2 / 3). On the
 *        STM32F411CEU6 the usual Black Pill wiring is:
 *          - I2C1 on PB6 (SCL) / PB7 (SDA)
 *          - I2C2 on PB10 (SCL) / PB3 (SDA)  — PB3 is SWO, avoid if
 *            you use SWD tracing
 *          - I2C3 on PA8 (SCL) / PB4 (SDA)
 */
#define MPU6050_I2C_CHANNEL   I2C_CHANNEL1

/**
 * @brief 7-bit I2C address of the MPU6050, right-aligned (no R/W bit).
 *
 * @note  0x68 when the AD0 pin is tied low (the usual default, e.g. on
 *        GY-521 boards), 0x69 when it's tied high. Change this to match
 *        your wiring.
 */
#define MPU6050_I2C_ADDR      0x68U

/** @brief Target SCL clock for the MPU6050 bus [Hz]. */
#define MPU6050_I2C_CLOCK_HZ  400000UL

/**
 * @brief APB1 peripheral clock feeding the I2C block [Hz].
 *
 * @note  Set this to the STM32F411CEU6's actual PCLK1 (from your RCC /
 *        clock tree config) — it feeds directly into the I2C driver's
 *        CCR / TRISE calculation, so a wrong value here mis-times the
 *        bus. At 100 MHz SYSCLK with APB1 divided by 2, PCLK1 = 50 MHz.
 */
#define MPU6050_I2C_PCLK1_HZ  50000000UL

/**
 * @brief Timer used for the driver's microsecond / millisecond delays.
 *
 * @note  A basic timer (TIM6 or TIM7) is the right choice — it has no
 *        output pins to waste, and nothing else in the firmware needs
 *        one.
 */
#define MPU6050_DELAY_TIMER   TIM_TIMER6

/** @} */

/*============================================================================*
 *                            FULL-SCALE RANGES                               *
 *============================================================================*/

/**
 * @name Full-scale ranges
 *
 * A narrower range gives finer resolution but saturates sooner. A road
 * vehicle never pulls multiple g or spins at hundreds of degrees per
 * second, so the narrowest range of each is the right trade.
 * @{
 */

/** @brief Accelerometer range: 0 = ±2 g, 1 = ±4 g, 2 = ±8 g, 3 = ±16 g. */
#define MPU6050_ACCEL_FS   0

/** @brief Gyroscope range: 0 = ±250 °/s, 1 = ±500, 2 = ±1000, 3 = ±2000. */
#define MPU6050_GYRO_FS    0

/** @} */

/*============================================================================*
 *                        YAW (HEADING) INTEGRATION                           *
 *============================================================================*/

/**
 * @name Yaw (heading) integration
 *
 * The MPU6050 has **no magnetometer**, so there is no absolute reference
 * to correct the gyroscope against — MPU6050_enumGetHeading is a pure
 * integration of GyroZ and, like any dead-reckoned angle, it drifts
 * without bound. It is good for short-horizon relative turns; it is not
 * a compass.
 * @{
 */

/**
 * @brief Sign of the gyroscope's contribution to the integrated
 *        heading: +1.0f or -1.0f.
 *
 * @note  Flip this if a steady turn one way makes the reported heading
 *        count down instead of up (or vice-versa) relative to which
 *        way you actually turned.
 */
#define MPU6050_GYROZ_SIGN   (+1.0f)

/** @} */

/** @} */ /* end of hal_mpu6050 */

#endif /* MPU6050_CONFIG_H */
