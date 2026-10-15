/**
 ******************************************************************************
 * @file    lsm303d_config.h
 * @brief   Compile-time configuration and defaults for the LSM303D driver.
 * @details
 *   Board wiring defaults, bus interface choices, and standard operating
 *   parameters for the STMicroelectronics LSM303D 6-axis eCompass.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef LSM303D_CONFIG_H
#define LSM303D_CONFIG_H

#include "lsm303d.h"

/*============================================================================*
 *                              WIRING DEFAULTS                               *
 *============================================================================*/

/**
 * @brief Default communication bus type:
 *   - LSM303D_BUS_SPI (using shared SPI1 bus on PA5/PA6/PA7)
 *   - LSM303D_BUS_I2C (using shared I2C1 bus on PB6/PB7)
 */
#ifndef LSM303D_DEFAULT_BUS
#define LSM303D_DEFAULT_BUS          LSM303D_BUS_SPI
#endif

/**
 * @brief Default GPIOA Chip Select pin for SPI mode.
 *        Defaults to PA0 on STM32F411CEU6.
 */
#ifndef LSM303D_DEFAULT_SPI_CS_PIN
#define LSM303D_DEFAULT_SPI_CS_PIN   0U
#endif

/**
 * @brief Default 7-bit I2C slave address.
 *        0x1E when SDO/SA0 is pulled low to GND (default on most breakout boards).
 *        0x1D when SDO/SA0 is pulled high to VDD.
 */
#ifndef LSM303D_DEFAULT_I2C_ADDR
#define LSM303D_DEFAULT_I2C_ADDR     LSM303D_I2C_ADDR_LOW
#endif

/*============================================================================*
 *                        SENSOR TUNING DEFAULTS                              *
 *============================================================================*/

/** @brief Default accelerometer full-scale dynamic range (±2 g). */
#define LSM303D_DEFAULT_ACCEL_FS     LSM303D_ACCEL_FS_2G

/** @brief Default accelerometer output data rate (100 Hz). */
#define LSM303D_DEFAULT_ACCEL_ODR    LSM303D_ACCEL_ODR_100HZ

/** @brief Default accelerometer anti-alias filter bandwidth (50 Hz). */
#define LSM303D_DEFAULT_ACCEL_BW     LSM303D_ACCEL_BW_50HZ

/** @brief Default magnetometer full-scale range (±4 Gauss). */
#define LSM303D_DEFAULT_MAG_FS       LSM303D_MAG_FS_4GAUSS

/** @brief Default magnetometer output data rate (100 Hz). */
#define LSM303D_DEFAULT_MAG_ODR      LSM303D_MAG_ODR_100HZ

#endif /* LSM303D_CONFIG_H */
