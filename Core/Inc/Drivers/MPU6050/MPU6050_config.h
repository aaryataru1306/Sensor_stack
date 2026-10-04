/**
 ******************************************************************************
 * @file    MPU6050_config.h
 * @author  Alaa Hassan
 * @brief   Compile-time tuning for the MPU6050 driver: wiring, ranges, and the
 *          sign of the yaw-integration term.
 * @ingroup hal_mpu6050
 ******************************************************************************
 */

#ifndef MPU6050_CONFIG_H_
#define MPU6050_CONFIG_H_

/**
 * @addtogroup hal_mpu6050
 * @{
 */

/**
 * @name Wiring
 *
 * MPU6050 talks to the STM32F411CEU6 over its main I2C bus.
 * @{
 */
/** @brief The I2C peripheral the IMU hangs off (I2C_CHANNEL1/2/3 on the STM32F411CEU6). */
#define MPU6050_I2C_CHANNEL I2C_CHANNEL1

/**
 * @brief 7-bit I2C address of the MPU6050, right-aligned (no R/W bit).
 * @note  0x68 when the AD0 pin is tied low (the usual default, e.g. on GY-521
 *        boards), 0x69 when it's tied high. Change this to match your wiring.
 */
#define MPU6050_I2C_ADDR 0x68U

/** @brief Target SCL clock for the MPU6050 bus [Hz]. */
#define MPU6050_I2C_CLOCK_HZ (400000UL)

/**
 * @brief APB1 peripheral clock feeding the I2C block [Hz].
 * @note  Set this to the STM32F411CEU6's actual PCLK1 (from your RCC/clock
 *        tree config) — it feeds directly into the I2C driver's CCR/TRISE
 *        calculation, so a wrong value here mis-times the bus.
 */
#define MPU6050_I2C_PCLK1_HZ (16000000UL)

/**
 * @brief Timer used for the driver's microsecond/millisecond delays.
 * @note  A basic timer (TIM6 or TIM7) is the right choice — it has no output
 *        pins to waste, and nothing else in the firmware needs one.
 */
#define MPU6050_DELAY_TIMER TIM_TIMER6
/** @} */

/**
 * @name Full-scale ranges
 *
 * A narrower range gives finer resolution but saturates sooner. A road vehicle
 * never pulls multiple g or spins at hundreds of degrees per second, so the
 * narrowest range of each is the right trade.
 * @{
 */
/** @brief Accelerometer range: 0 = ±2 g, 1 = ±4 g, 2 = ±8 g, 3 = ±16 g. */
#define MPU6050_ACCEL_FS 0

/** @brief Gyroscope range: 0 = ±250 °/s, 1 = ±500, 2 = ±1000, 3 = ±2000. */
#define MPU6050_GYRO_FS 0
/** @} */

/**
 * @name Yaw (heading) integration
 *
 * The MPU6050 has **no magnetometer**, so there is no absolute reference to
 * correct the gyroscope against — @ref MPU6050_enumGetHeading is a pure
 * integration of GyroZ and, like any dead-reckoned angle, it drifts without
 * bound. It is good for short-horizon relative turns; it is not a compass.
 * @{
 */
/**
 * @brief Sign of the gyroscope's contribution to the integrated heading: +1.0f or -1.0f.
 *
 * Flip this if a steady turn one way makes the reported heading count down
 * instead of up (or vice-versa) relative to which way you actually turned.
 */
#define MPU6050_GYROZ_SIGN (+1.0f)
/** @} */

/** @} */ /* end of hal_mpu6050 */

#endif /* MPU6050_CONFIG_H_ */
