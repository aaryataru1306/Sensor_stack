/**
 ******************************************************************************
 * @file    icm20948_private.h
 * @brief   ICM-20948 register map, bit values, and scale factors —
 *          driver-internal.
 * @details
 *   Register addresses and bank layout from the TDK InvenSense
 *   ICM-20948 datasheet (DS-000189) and register map (RM-000108).
 *
 *   Nothing here is part of the driver's public API; include
 *   icm20948.h instead.
 *
 * @note  The ICM-20948 is a 9-axis part (accel + gyro + AK09916
 *        magnetometer), but this driver only exposes the accel, gyro
 *        and die temperature. The magnetometer lives behind an internal
 *        I2C master on the ICM and needs a separate driver — not
 *        implemented here.
 *
 * @note  The ICM-20948 splits its register space into 4 banks, selected
 *        via REG_BANK_SEL (0x7F, same address in every bank). The bank
 *        MUST be re-selected after a reset, since the bank state is not
 *        preserved.
 *
 * @note  Multi-byte sensor values are big-endian: `(H << 8) | L`.
 ******************************************************************************
 */

#ifndef ICM20948_PRIVATE_H
#define ICM20948_PRIVATE_H

#include <stdint.h>

/*============================================================================*
 *                       REGISTER BANK SELECT (all banks)                     *
 *============================================================================*/

#define ICM20948_REG_BANK_SEL   0x7FU   /**< bits[5:4] = bank (0..3). */

/*============================================================================*
 *                          BANK 0 — CONFIG / DATA                             *
 *============================================================================*/

#define ICM20948_B0_WHO_AM_I        0x00U   /**< Reads 0xEA on genuine part. */
#define ICM20948_B0_REVISION        0x01U
#define ICM20948_B0_USER_CTRL       0x03U   /**< bit5 = I2C_MST_EN, bit4 = FIFO_EN,
                                                 bit2 = SRAM_RST, bit0 = DMP_RST. */
#define ICM20948_B0_LP_CONFIG       0x05U
#define ICM20948_B0_PWR_MGMT_1      0x06U   /**< bit7 = DEVICE_RESET, bit6 = SLEEP,
                                                 bit5 = LP_CYCLE, bits[3:0] = CLKSEL. */
#define ICM20948_B0_PWR_MGMT_2      0x07U   /**< bits[5:3] = DISABLE_ACCEL,
                                                 bits[2:0] = DISABLE_GYRO. */
#define ICM20948_B0_INT_PIN_CFG     0x0FU
#define ICM20948_B0_INT_ENABLE      0x10U
#define ICM20948_B0_INT_ENABLE_1    0x11U
#define ICM20948_B0_INT_ENABLE_2    0x12U
#define ICM20948_B0_INT_ENABLE_3    0x13U
#define ICM20948_B0_I2C_MST_STATUS  0x17U
#define ICM20948_B0_INT_STATUS      0x19U
#define ICM20948_B0_INT_STATUS_1    0x1AU
#define ICM20948_B0_INT_STATUS_2    0x1BU
#define ICM20948_B0_INT_STATUS_3    0x1CU
#define ICM20948_B0_DELAY_TIMEH     0x28U
#define ICM20948_B0_DELAY_TIMEL     0x29U
#define ICM20948_B0_ACCEL_XOUT_H    0x2DU   /**< Start of 20-byte burst:
                                                 accel(6) + gyro(6) + temp(2) + ... */
#define ICM20948_B0_ACCEL_XOUT_L    0x2EU
#define ICM20948_B0_ACCEL_YOUT_H    0x2FU
#define ICM20948_B0_ACCEL_YOUT_L    0x30U
#define ICM20948_B0_ACCEL_ZOUT_H    0x31U
#define ICM20948_B0_ACCEL_ZOUT_L    0x32U
#define ICM20948_B0_GYRO_XOUT_H     0x33U   /**< Start of 6-byte gyro burst. */
#define ICM20948_B0_GYRO_XOUT_L     0x34U
#define ICM20948_B0_GYRO_YOUT_H     0x35U
#define ICM20948_B0_GYRO_YOUT_L     0x36U
#define ICM20948_B0_GYRO_ZOUT_H     0x37U
#define ICM20948_B0_GYRO_ZOUT_L     0x38U
#define ICM20948_B0_TEMP_OUT_H      0x39U   /**< 16-bit big-endian die temp. */
#define ICM20948_B0_TEMP_OUT_L      0x3AU
#define ICM20948_B0_EXT_SLV_SENS_DATA_00 0x3BU
#define ICM20948_B0_FIFO_EN_1       0x66U
#define ICM20948_B0_FIFO_EN_2       0x67U
#define ICM20948_B0_FIFO_RST        0x68U
#define ICM20948_B0_FIFO_MODE       0x69U
#define ICM20948_B0_FIFO_COUNTH     0x70U
#define ICM20948_B0_FIFO_COUNTL     0x71U
#define ICM20948_B0_FIFO_R_W        0x72U
#define ICM20948_B0_DATA_RDY_STATUS 0x74U
#define ICM20948_B0_FIFO_CFG        0x76U

/*============================================================================*
 *                    BANK 1 — USER BANK (unused by this driver)              *
 *============================================================================*
 * Reserved for DMP / self-test / sensor-config extensions. Not touched
 * by the current driver — declared here only to document the bank split.
 */

/*============================================================================*
 *                          BANK 2 — CONFIGURATION                            *
 *============================================================================*/

#define ICM20948_B2_GYRO_SMPLRT_DIV      0x00U   /**< Gyro sample-rate divider. */
#define ICM20948_B2_GYRO_CONFIG_1        0x01U   /**< bit[5:3] = GYRO_DLPFCFG,
                                                       bit[2:1] = GYRO_FS_SEL,
                                                       bit0 = GYRO_FCHOICE. */
#define ICM20948_B2_GYRO_CONFIG_2        0x02U
#define ICM20948_B2_XG_OFFS_USRH         0x03U
#define ICM20948_B2_XG_OFFS_USRL         0x04U
#define ICM20948_B2_YG_OFFS_USRH         0x05U
#define ICM20948_B2_YG_OFFS_USRL         0x06U
#define ICM20948_B2_ZG_OFFS_USRH         0x07U
#define ICM20948_B2_ZG_OFFS_USRL         0x08U
#define ICM20948_B2_ODR_ALIGN_EN         0x09U
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1   0x10U   /**< high byte. */
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2   0x11U   /**< low byte. */
#define ICM20948_B2_ACCEL_INTEL_CTRL     0x12U
#define ICM20948_B2_ACCEL_WOM_THR        0x13U
#define ICM20948_B2_ACCEL_CONFIG         0x14U   /**< bit[5:3] = ACCEL_DLPFCFG,
                                                       bit[2:1] = ACCEL_FS_SEL,
                                                       bit0 = ACCEL_FCHOICE. */
#define ICM20948_B2_ACCEL_CONFIG_2       0x15U
#define ICM20948_B2_FSYNC_CONFIG         0x52U
#define ICM20948_B2_TEMP_CONFIG          0x53U
#define ICM20948_B2_MOD_CTRL_USR         0x54U

/*============================================================================*
 *                          BANK 3 — I2C MASTER (unused)                      *
 *============================================================================*
 * Internal I2C master registers for the AK09916 magnetometer. Not
 * implemented — a future magnetometer driver would add them here.
 */

/*============================================================================*
 *                          REGISTER / CONSTANT VALUES                        *
 *============================================================================*/

#define ICM20948_WHO_AM_I_VALUE     0xEAU   /**< Expected WHO_AM_I. */

/* PWR_MGMT_1 bits */
#define ICM20948_PWR_DEVICE_RESET   (1U << 7)
#define ICM20948_PWR_SLEEP          (1U << 6)
#define ICM20948_PWR_LP_CYCLE       (1U << 5)
#define ICM20948_CLKSEL_AUTO        0x01U   /**< Auto-select best clock (PLL if
                                                 available). */
#define ICM20948_CLKSEL_INTERNAL    0x00U   /**< Internal 20 MHz oscillator. */

/* USER_CTRL bits */
#define ICM20948_USERCTRL_DMP_EN    (1U << 7)
#define ICM20948_USERCTRL_FIFO_EN   (1U << 6)
#define ICM20948_USERCTRL_I2C_MST_EN (1U << 5)
#define ICM20948_USERCTRL_I2C_IF_DIS (1U << 4) /**< Force SPI-only mode. */

/*============================================================================*
 *                       FULL-SCALE RANGE ENUM VALUES                         *
 *============================================================================*/

/* GYRO_FS_SEL field in GYRO_CONFIG_1[2:1] */
#define ICM20948_GYRO_FS_250DPS_VAL   0x00U
#define ICM20948_GYRO_FS_500DPS_VAL   0x01U
#define ICM20948_GYRO_FS_1000DPS_VAL  0x02U
#define ICM20948_GYRO_FS_2000DPS_VAL  0x03U

/* ACCEL_FS_SEL field in ACCEL_CONFIG[2:1] */
#define ICM20948_ACCEL_FS_2G_VAL      0x00U
#define ICM20948_ACCEL_FS_4G_VAL      0x01U
#define ICM20948_ACCEL_FS_8G_VAL      0x02U
#define ICM20948_ACCEL_FS_16G_VAL     0x03U

/*============================================================================*
 *                          SENSITIVITY SCALE FACTORS                         *
 *============================================================================*/

#define ICM20948_ACCEL_SENS_2G    16384.0f   /**< LSB/g  at ±2 g.  */
#define ICM20948_ACCEL_SENS_4G    8192.0f    /**< LSB/g  at ±4 g.  */
#define ICM20948_ACCEL_SENS_8G    4096.0f    /**< LSB/g  at ±8 g.  */
#define ICM20948_ACCEL_SENS_16G   2048.0f    /**< LSB/g  at ±16 g. */

#define ICM20948_GYRO_SENS_250DPS   131.0f   /**< LSB/dps at ±250 dps.  */
#define ICM20948_GYRO_SENS_500DPS   65.5f    /**< LSB/dps at ±500 dps.  */
#define ICM20948_GYRO_SENS_1000DPS  32.8f    /**< LSB/dps at ±1000 dps. */
#define ICM20948_GYRO_SENS_2000DPS  16.4f    /**< LSB/dps at ±2000 dps. */

/*============================================================================*
 *                          TEMPERATURE CONVERSION                            *
 *============================================================================*/

/** @brief Temperature [°C] = (raw / 333.87) + 21.0   (datasheet §4.9). */
#define ICM20948_TEMP_SENSITIVITY  333.87f
#define ICM20948_TEMP_OFFSET_C     21.0f

#endif /* ICM20948_PRIVATE_H */
