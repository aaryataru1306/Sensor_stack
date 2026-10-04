#ifndef ICM20948_CONFIG_H
#define ICM20948_CONFIG_H

/*============================================================================*
 *                            HARDWARE PIN MAPPING                            *
 *============================================================================*/

/**
 * @brief Chip Select Pin for ICM20948 on SPI1.
 * @note  On the STM32F411 Black Pill, PA3 is default for ICM20948 CS.
 */
#define ICM20948_CS_PIN               3U

/*============================================================================*
 *                        DEFAULT SENSOR CONFIGURATION                        *
 *============================================================================*/

/**
 * @brief Default Accelerometer Full-Scale Range (±2g default for vehicles/ground).
 */
#define ICM20948_DEFAULT_ACCEL_FS     ICM20948_ACCEL_FS_2G

/**
 * @brief Default Gyroscope Full-Scale Range (±250 dps default).
 */
#define ICM20948_DEFAULT_GYRO_FS      ICM20948_GYRO_FS_250DPS

#endif /* ICM20948_CONFIG_H */