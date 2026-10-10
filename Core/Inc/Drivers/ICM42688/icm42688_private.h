/**
 ******************************************************************************
 * @file    icm42688_private.h
 * @brief   Register map, bitfield definitions, and scale factors for the
 *          TDK InvenSense ICM-42688-P 6-axis High-Performance IMU.
 * @details
 *   Based on TDK InvenSense ICM-42688-P Datasheet (DS-000347) and
 *   Register Map (RM-000388).
 *
 *   The ICM-42688-P organizes its registers into 5 banks (Bank 0 to Bank 4),
 *   selected via REG_BANK_SEL (0x76).
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef ICM42688_PRIVATE_H
#define ICM42688_PRIVATE_H

#include <stdint.h>

/*============================================================================*
 *                           BANK SELECT REGISTER                             *
 *============================================================================*/

#define ICM42688_REG_BANK_SEL           0x76U   /**< Bank select in all banks: bits[2:0] = 0..4 */
#define ICM42688_BANK_0                 0x00U
#define ICM42688_BANK_1                 0x01U
#define ICM42688_BANK_2                 0x02U
#define ICM42688_BANK_3                 0x03U
#define ICM42688_BANK_4                 0x04U

/*============================================================================*
 *                           USER BANK 0 REGISTERS                            *
 *============================================================================*/

#define ICM42688_B0_DEVICE_CONFIG       0x11U   /**< bit0 = SOFT_RESET_CONFIG */
#define ICM42688_B0_DRIVE_CONFIG        0x13U   /**< Slew rates for I2C/SPI */
#define ICM42688_B0_INT_CONFIG          0x14U   /**< Interrupt pin polarity & drive */
#define ICM42688_B0_FIFO_CONFIG         0x16U   /**< FIFO mode (Bypass, Stream, Stop-on-full) */

/* Data Output Registers (14 consecutive bytes starting at 0x1D) */
#define ICM42688_B0_TEMP_DATA1          0x1DU   /**< Temperature [15:8] */
#define ICM42688_B0_TEMP_DATA0          0x1EU   /**< Temperature [7:0] */
#define ICM42688_B0_ACCEL_DATA_X1       0x1FU   /**< Accel X [15:8] */
#define ICM42688_B0_ACCEL_DATA_X0       0x20U   /**< Accel X [7:0] */
#define ICM42688_B0_ACCEL_DATA_Y1       0x21U   /**< Accel Y [15:8] */
#define ICM42688_B0_ACCEL_DATA_Y0       0x22U   /**< Accel Y [7:0] */
#define ICM42688_B0_ACCEL_DATA_Z1       0x23U   /**< Accel Z [15:8] */
#define ICM42688_B0_ACCEL_DATA_Z0       0x24U   /**< Accel Z [7:0] */
#define ICM42688_B0_GYRO_DATA_X1        0x25U   /**< Gyro X [15:8] */
#define ICM42688_B0_GYRO_DATA_X0        0x26U   /**< Gyro X [7:0] */
#define ICM42688_B0_GYRO_DATA_Y1        0x27U   /**< Gyro Y [15:8] */
#define ICM42688_B0_GYRO_DATA_Y0        0x28U   /**< Gyro Y [7:0] */
#define ICM42688_B0_GYRO_DATA_Z1        0x29U   /**< Gyro Z [15:8] */
#define ICM42688_B0_GYRO_DATA_Z0        0x2AU   /**< Gyro Z [7:0] */
#define ICM42688_B0_TMST_FSYNCH         0x2BU
#define ICM42688_B0_TMST_FSYNCL         0x2CU

/* Status & FIFO */
#define ICM42688_B0_INT_STATUS          0x2DU   /**< bit3 = DATA_RDY_INT */
#define ICM42688_B0_FIFO_COUNTH         0x2EU
#define ICM42688_B0_FIFO_COUNTL         0x2FU
#define ICM42688_B0_FIFO_DATA           0x30U
#define ICM42688_B0_APEX_DATA0          0x31U
#define ICM42688_B0_APEX_DATA1          0x32U
#define ICM42688_B0_APEX_DATA2          0x33U
#define ICM42688_B0_APEX_DATA3          0x34U
#define ICM42688_B0_APEX_DATA4          0x35U
#define ICM42688_B0_APEX_DATA5          0x36U
#define ICM42688_B0_APEX_STATUS0        0x37U
#define ICM42688_B0_APEX_STATUS1        0x38U
#define ICM42688_B0_INT_STATUS2         0x39U
#define ICM42688_B0_INT_STATUS3         0x3AU

/* Signal Path & Interface Config */
#define ICM42688_B0_SIGNAL_PATH_RESET   0x4BU   /**< bit1 = TMST_STROBE, bit0 = FIFO_FLUSH */
#define ICM42688_B0_INTF_CONFIG0        0x4CU   /**< bits[1:0] = UI_SIFS_CFG (disable I2C) */
#define ICM42688_B0_INTF_CONFIG1        0x4DU   /**< bits[1:0] = CLKSEL */
#define ICM42688_B0_PWR_MGMT0           0x4EU   /**< bits[3:2]=GYRO_MODE, bits[1:0]=ACCEL_MODE */
#define ICM42688_B0_GYRO_CONFIG0        0x4FU   /**< bits[7:5]=FS_SEL, bits[3:0]=ODR */
#define ICM42688_B0_ACCEL_CONFIG0       0x50U   /**< bits[7:5]=FS_SEL, bits[3:0]=ODR */
#define ICM42688_B0_GYRO_CONFIG1        0x51U   /**< UI filter order */
#define ICM42688_B0_GYRO_ACCEL_CONFIG0  0x52U   /**< UI filter bandwidth (DLPF) */
#define ICM42688_B0_ACCEL_CONFIG1       0x53U   /**< Accel UI filter order */
#define ICM42688_B0_TMST_CONFIG         0x54U   /**< Timestamp enable & resolution */
#define ICM42688_B0_APEX_CONFIG0        0x56U
#define ICM42688_B0_SMD_CONFIG          0x57U
#define ICM42688_B0_FIFO_CONFIG1        0x5FU
#define ICM42688_B0_FIFO_CONFIG2        0x60U
#define ICM42688_B0_FIFO_CONFIG3        0x61U
#define ICM42688_B0_FSYNC_CONFIG        0x62U
#define ICM42688_B0_INT_CONFIG0         0x63U
#define ICM42688_B0_INT_CONFIG1         0x64U
#define ICM42688_B0_INT_SOURCE0         0x65U   /**< Map UI data ready to INT1 */
#define ICM42688_B0_INT_SOURCE1         0x66U
#define ICM42688_B0_INT_SOURCE3         0x68U
#define ICM42688_B0_INT_SOURCE4         0x69U
#define ICM42688_B0_FIFO_LOST_PKT0      0x6CU
#define ICM42688_B0_FIFO_LOST_PKT1      0x6DU
#define ICM42688_B0_SELF_TEST_CONFIG    0x70U
#define ICM42688_B0_WHO_AM_I            0x75U   /**< Expected value: 0x47 */

/*============================================================================*
 *                           USER BANK 1 REGISTERS                            *
 *============================================================================*/

#define ICM42688_B1_SENSOR_CONFIG0      0x03U
#define ICM42688_B1_GYRO_CONFIG_STATIC2 0x0BU   /**< Anti-aliasing filter (AAF) */
#define ICM42688_B1_GYRO_CONFIG_STATIC3 0x0CU
#define ICM42688_B1_GYRO_CONFIG_STATIC4 0x0DU
#define ICM42688_B1_GYRO_CONFIG_STATIC5 0x0EU
#define ICM42688_B1_INTF_CONFIG5        0x7BU

/*============================================================================*
 *                           USER BANK 2 REGISTERS                            *
 *============================================================================*/

#define ICM42688_B2_ACCEL_CONFIG_STATIC2 0x03U  /**< Accel Anti-aliasing filter (AAF) */
#define ICM42688_B2_ACCEL_CONFIG_STATIC3 0x04U
#define ICM42688_B2_ACCEL_CONFIG_STATIC4 0x05U

/*============================================================================*
 *                       REGISTER BITFIELD DEFINITIONS                        *
 *============================================================================*/

/* DEVICE_CONFIG (0x11) */
#define ICM42688_SOFT_RESET_BIT         (1U << 0)
#define ICM42688_SPI_MODE_BIT           (1U << 4)

/* WHO_AM_I Value */
#define ICM42688_WHO_AM_I_VALUE         0x47U

/* INT_CONFIG (0x14) */
#define ICM42688_INT1_POLARITY_HIGH     (1U << 0)
#define ICM42688_INT1_DRIVE_PUSHPULL    (1U << 1)
#define ICM42688_INT1_MODE_LATCHED      (1U << 2)

/* INT_STATUS (0x2D) */
#define ICM42688_INT_STATUS_DATA_RDY    (1U << 3)
#define ICM42688_INT_STATUS_FIFO_THS    (1U << 2)
#define ICM42688_INT_STATUS_FIFO_FULL   (1U << 1)

/* INTF_CONFIG0 (0x4C) */
#define ICM42688_UI_SIFS_DISABLE_I2C    (3U << 0)  /**< Lock to SPI, disable I2C */
#define ICM42688_SENSOR_DATA_LITTLE_END (0U << 4)
#define ICM42688_FIFO_COUNT_LITTLE_END  (0U << 5)

/* INTF_CONFIG1 (0x4D) */
#define ICM42688_CLKSEL_PLL_OR_RC       (1U << 0)  /**< Auto select PLL when available */

/* PWR_MGMT0 (0x4E) */
#define ICM42688_PWR_TEMP_ON            (0U << 5)
#define ICM42688_PWR_TEMP_OFF           (1U << 5)
#define ICM42688_PWR_IDLE_RC_ON         (1U << 4)

#define ICM42688_GYRO_MODE_OFF          (0U << 2)
#define ICM42688_GYRO_MODE_STANDBY      (1U << 2)
#define ICM42688_GYRO_MODE_LOW_NOISE    (3U << 2)  /**< Enables 2.8 mdps/√Hz LN mode */

#define ICM42688_ACCEL_MODE_OFF         (0U << 0)
#define ICM42688_ACCEL_MODE_LOW_POWER   (2U << 0)
#define ICM42688_ACCEL_MODE_LOW_NOISE   (3U << 0)  /**< Low-Noise mode */

/* INT_SOURCE0 (0x65) */
#define ICM42688_INT_UI_DRDY_INT1_EN    (1U << 3)

/*============================================================================*
 *                   SCALE FACTORS & PHYSICAL CONVERSION                      *
 *============================================================================*/

/**
 * @name Gyroscope Sensitivity Scale Factors [LSB / (°/s)]
 * @details 16-bit signed integer full-scale range conversions.
 */
#define ICM42688_GYRO_SENS_2000DPS      16.384f  /**< ±2000 dps -> 16.4 LSB/(°/s) */
#define ICM42688_GYRO_SENS_1000DPS      32.768f  /**< ±1000 dps -> 32.8 LSB/(°/s) */
#define ICM42688_GYRO_SENS_500DPS       65.536f  /**< ±500 dps  -> 65.5 LSB/(°/s) */
#define ICM42688_GYRO_SENS_250DPS       131.072f /**< ±250 dps  -> 131.0 LSB/(°/s) */
#define ICM42688_GYRO_SENS_125DPS       262.144f /**< ±125 dps  -> 262.0 LSB/(°/s) */
#define ICM42688_GYRO_SENS_62_5DPS      524.288f /**< ±62.5 dps -> 524.3 LSB/(°/s) */
#define ICM42688_GYRO_SENS_31_25DPS     1048.576f/**< ±31.25 dps-> 1048.6 LSB/(°/s) */
#define ICM42688_GYRO_SENS_15_625DPS    2097.152f/**< ±15.625 dps->2097.2 LSB/(°/s) */

/**
 * @name Accelerometer Sensitivity Scale Factors [LSB / g]
 */
#define ICM42688_ACCEL_SENS_16G         2048.0f  /**< ±16 g -> 2,048 LSB/g */
#define ICM42688_ACCEL_SENS_8G          4096.0f  /**< ±8 g  -> 4,096 LSB/g */
#define ICM42688_ACCEL_SENS_4G          8192.0f  /**< ±4 g  -> 8,192 LSB/g */
#define ICM42688_ACCEL_SENS_2G          16384.0f /**< ±2 g  -> 16,384 LSB/g */

/**
 * @name Temperature Sensor Conversion
 * @details Temperature in °C = (TEMP_DATA / 132.48) + 25.0
 */
#define ICM42688_TEMP_SENSITIVITY       132.48f  /**< LSB / °C */
#define ICM42688_TEMP_OFFSET_C          25.0f    /**< °C offset */

#endif /* ICM42688_PRIVATE_H */
