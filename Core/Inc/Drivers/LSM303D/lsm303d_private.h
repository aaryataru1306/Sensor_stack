/**
 ******************************************************************************
 * @file    lsm303d_private.h
 * @brief   Register map, bit-field definitions, and hardware constants for the
 *          STMicroelectronics LSM303D 3D accelerometer and 3D magnetometer module.
 * @details
 *   Based directly on STMicroelectronics LSM303D Datasheet (DocID023312 Rev 2):
 *     - Device ID (WHO_AM_I): 0x49
 *     - Standard 7-bit I2C addresses:
 *         0x1E (SDO/SA0 connected to GND)
 *         0x1D (SDO/SA0 connected to VDD)
 *     - Multi-byte auto-increment:
 *         I2C: Sub-address bit 7 = 1 (SUB | 0x80)
 *         SPI: Command byte bit 7 (RW) = 1, bit 6 (MS) = 1 (0xC0 | (reg & 0x3F))
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef LSM303D_PRIVATE_H
#define LSM303D_PRIVATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        REGISTER MAP DEFINITIONS                            *
 *============================================================================*/

#define LSM303D_REG_TEMP_OUT_L         0x05U  /**< Temperature sensor output data low byte */
#define LSM303D_REG_TEMP_OUT_H         0x06U  /**< Temperature sensor output data high byte */
#define LSM303D_REG_STATUS_M           0x07U  /**< Magnetic status register */
#define LSM303D_REG_OUT_X_L_M          0x08U  /**< Magnetometer X-axis output low byte */
#define LSM303D_REG_OUT_X_H_M          0x09U  /**< Magnetometer X-axis output high byte */
#define LSM303D_REG_OUT_Y_L_M          0x0AU  /**< Magnetometer Y-axis output low byte */
#define LSM303D_REG_OUT_Y_H_M          0x0BU  /**< Magnetometer Y-axis output high byte */
#define LSM303D_REG_OUT_Z_L_M          0x0CU  /**< Magnetometer Z-axis output low byte */
#define LSM303D_REG_OUT_Z_H_M          0x0DU  /**< Magnetometer Z-axis output high byte */
#define LSM303D_REG_WHO_AM_I           0x0FU  /**< Device identification register */

#define LSM303D_REG_INT_CTRL_M         0x12U  /**< Magnetometer interrupt control register */
#define LSM303D_REG_INT_SRC_M          0x13U  /**< Magnetometer interrupt source register */
#define LSM303D_REG_INT_THS_L_M        0x14U  /**< Magnetometer interrupt threshold low byte */
#define LSM303D_REG_INT_THS_H_M        0x15U  /**< Magnetometer interrupt threshold high byte */

#define LSM303D_REG_OFFSET_X_L_M       0x16U  /**< Magnetometer user offset X low byte */
#define LSM303D_REG_OFFSET_X_H_M       0x17U  /**< Magnetometer user offset X high byte */
#define LSM303D_REG_OFFSET_Y_L_M       0x18U  /**< Magnetometer user offset Y low byte */
#define LSM303D_REG_OFFSET_Y_H_M       0x19U  /**< Magnetometer user offset Y high byte */
#define LSM303D_REG_OFFSET_Z_L_M       0x1AU  /**< Magnetometer user offset Z low byte */
#define LSM303D_REG_OFFSET_Z_H_M       0x1BU  /**< Magnetometer user offset Z high byte */

#define LSM303D_REG_REFERENCE_X        0x1CU  /**< Acceleration reference X register */
#define LSM303D_REG_REFERENCE_Y        0x1DU  /**< Acceleration reference Y register */
#define LSM303D_REG_REFERENCE_Z        0x1EU  /**< Acceleration reference Z register */

#define LSM303D_REG_CTRL_REG0          0x1FU  /**< Control register 0: FIFO, boot, filter */
#define LSM303D_REG_CTRL_REG1          0x20U  /**< Control register 1: Accel ODR, BDU, axes enable */
#define LSM303D_REG_CTRL_REG2          0x21U  /**< Control register 2: Anti-alias filter, Accel FS */
#define LSM303D_REG_CTRL_REG3          0x22U  /**< Control register 3: INT1 interrupt pin control */
#define LSM303D_REG_CTRL_REG4          0x23U  /**< Control register 4: INT2 interrupt pin control */
#define LSM303D_REG_CTRL_REG5          0x24U  /**< Control register 5: Temp enable, Mag resolution, Mag ODR */
#define LSM303D_REG_CTRL_REG6          0x25U  /**< Control register 6: Mag full-scale selection */
#define LSM303D_REG_CTRL_REG7          0x26U  /**< Control register 7: Mag operating mode, HP filter */

#define LSM303D_REG_STATUS_A           0x27U  /**< Accelerometer status register */
#define LSM303D_REG_OUT_X_L_A          0x28U  /**< Accelerometer X-axis output low byte */
#define LSM303D_REG_OUT_X_H_A          0x29U  /**< Accelerometer X-axis output high byte */
#define LSM303D_REG_OUT_Y_L_A          0x2AU  /**< Accelerometer Y-axis output low byte */
#define LSM303D_REG_OUT_Y_H_A          0x2BU  /**< Accelerometer Y-axis output high byte */
#define LSM303D_REG_OUT_Z_L_A          0x2CU  /**< Accelerometer Z-axis output low byte */
#define LSM303D_REG_OUT_Z_H_A          0x2DU  /**< Accelerometer Z-axis output high byte */

#define LSM303D_REG_FIFO_CTRL          0x2EU  /**< FIFO control register */
#define LSM303D_REG_FIFO_SRC           0x2FU  /**< FIFO status/source register */
#define LSM303D_REG_IG_CFG1            0x30U  /**< Interrupt generator 1 configuration */
#define LSM303D_REG_IG_SRC1            0x31U  /**< Interrupt generator 1 source */
#define LSM303D_REG_IG_THS1            0x32U  /**< Interrupt generator 1 threshold */
#define LSM303D_REG_IG_DUR1            0x33U  /**< Interrupt generator 1 duration */
#define LSM303D_REG_IG_CFG2            0x34U  /**< Interrupt generator 2 configuration */
#define LSM303D_REG_IG_SRC2            0x35U  /**< Interrupt generator 2 source */
#define LSM303D_REG_IG_THS2            0x36U  /**< Interrupt generator 2 threshold */
#define LSM303D_REG_IG_DUR2            0x37U  /**< Interrupt generator 2 duration */
#define LSM303D_REG_CLICK_CFG          0x38U  /**< Click interrupt configuration */
#define LSM303D_REG_CLICK_SRC          0x39U  /**< Click source */
#define LSM303D_REG_CLICK_THS          0x3AU  /**< Click threshold */
#define LSM303D_REG_TIME_LIMIT         0x3BU  /**< Click time limit */
#define LSM303D_REG_TIME_LATENCY       0x3CU  /**< Click latency window */
#define LSM303D_REG_TIME_WINDOW        0x3DU  /**< Click time window */
#define LSM303D_REG_ACT_THS            0x3EU  /**< Sleep-to-wake threshold */
#define LSM303D_REG_ACT_DUR            0x3FU  /**< Sleep-to-wake duration */

/*============================================================================*
 *                        HARDWARE & PROTOCOL CONSTANTS                       *
 *============================================================================*/

/** @brief Expected value returned by WHO_AM_I register (0x0F). */
#define LSM303D_WHO_AM_I_VAL           0x49U

/** @brief 7-bit I2C slave address when SDO/SA0 pin is tied low to GND. */
#define LSM303D_I2C_ADDR_LOW           0x1EU

/** @brief 7-bit I2C slave address when SDO/SA0 pin is pulled high to VDD. */
#define LSM303D_I2C_ADDR_HIGH          0x1DU

/** @brief Sub-address auto-increment bit for multi-byte I2C read/write transactions. */
#define LSM303D_I2C_AUTO_INC_MASK      0x80U

/** @brief SPI command byte Read/Write bit: 1 = Read, 0 = Write. */
#define LSM303D_SPI_READ_BIT           0x80U

/** @brief SPI command byte Multiple/Single address bit: 1 = Auto-increment, 0 = Single. */
#define LSM303D_SPI_MS_BIT             0x40U

/** @brief SPI command byte register address mask (6 bits). */
#define LSM303D_SPI_ADDR_MASK          0x3FU

/*============================================================================*
 *                     REGISTER BIT-FIELD DEFINITIONS                         *
 *============================================================================*/

/* --- CTRL_REG0 (0x1F) --- */
#define LSM303D_CTRL0_BOOT             (1U << 7)
#define LSM303D_CTRL0_FIFO_EN          (1U << 6)
#define LSM303D_CTRL0_WTM_EN           (1U << 5)
#define LSM303D_CTRL0_HP_CLICK         (1U << 4)
#define LSM303D_CTRL0_HPIS1            (1U << 3)
#define LSM303D_CTRL0_HPIS2            (1U << 2)

/* --- CTRL_REG1 (0x20) --- */
#define LSM303D_CTRL1_AODR_POS         4U
#define LSM303D_CTRL1_AODR_MASK        (0x0FU << LSM303D_CTRL1_AODR_POS)
#define LSM303D_CTRL1_BDU              (1U << 3)
#define LSM303D_CTRL1_AZEN             (1U << 2)
#define LSM303D_CTRL1_AYEN             (1U << 1)
#define LSM303D_CTRL1_AXEN             (1U << 0)
#define LSM303D_CTRL1_ALL_AXES         (LSM303D_CTRL1_AZEN | LSM303D_CTRL1_AYEN | LSM303D_CTRL1_AXEN)

/* --- CTRL_REG2 (0x21) --- */
#define LSM303D_CTRL2_ABW_POS          6U
#define LSM303D_CTRL2_ABW_MASK         (0x03U << LSM303D_CTRL2_ABW_POS)
#define LSM303D_CTRL2_AFS_POS          3U
#define LSM303D_CTRL2_AFS_MASK         (0x07U << LSM303D_CTRL2_AFS_POS)
#define LSM303D_CTRL2_AST_POS          1U
#define LSM303D_CTRL2_AST_MASK         (0x03U << LSM303D_CTRL2_AST_POS)
#define LSM303D_CTRL2_SIM_3WIRE        (1U << 0)

/* --- CTRL_REG5 (0x24) --- */
#define LSM303D_CTRL5_TEMP_EN          (1U << 7)
#define LSM303D_CTRL5_M_RES_POS        5U
#define LSM303D_CTRL5_M_RES_MASK       (0x03U << LSM303D_CTRL5_M_RES_POS)
#define LSM303D_CTRL5_M_RES_HIGH       (0x03U << LSM303D_CTRL5_M_RES_POS)
#define LSM303D_CTRL5_M_ODR_POS        2U
#define LSM303D_CTRL5_M_ODR_MASK       (0x07U << LSM303D_CTRL5_M_ODR_POS)
#define LSM303D_CTRL5_LIR2             (1U << 1)
#define LSM303D_CTRL5_LIR1             (1U << 0)

/* --- CTRL_REG6 (0x25) --- */
#define LSM303D_CTRL6_MFS_POS          5U
#define LSM303D_CTRL6_MFS_MASK         (0x03U << LSM303D_CTRL6_MFS_POS)

/* --- CTRL_REG7 (0x26) --- */
#define LSM303D_CTRL7_AHPM_POS         6U
#define LSM303D_CTRL7_AHPM_MASK        (0x03U << LSM303D_CTRL7_AHPM_POS)
#define LSM303D_CTRL7_AFDS             (1U << 5)
#define LSM303D_CTRL7_T_ONLY           (1U << 4)
#define LSM303D_CTRL7_MLP              (1U << 2)
#define LSM303D_CTRL7_MD_POS           0U
#define LSM303D_CTRL7_MD_MASK          (0x03U << LSM303D_CTRL7_MD_POS)
#define LSM303D_CTRL7_MD_CONTINUOUS    (0x00U << LSM303D_CTRL7_MD_POS)
#define LSM303D_CTRL7_MD_SINGLE        (0x01U << LSM303D_CTRL7_MD_POS)
#define LSM303D_CTRL7_MD_POWERDOWN     (0x02U << LSM303D_CTRL7_MD_POS)

/* --- STATUS_A (0x27) --- */
#define LSM303D_STATUS_A_ZYXAOW        (1U << 7)
#define LSM303D_STATUS_A_ZAOW          (1U << 6)
#define LSM303D_STATUS_A_YAOW          (1U << 5)
#define LSM303D_STATUS_A_XAOW          (1U << 4)
#define LSM303D_STATUS_A_ZYXADA        (1U << 3)  /**< Accel new data ready for X, Y, Z */
#define LSM303D_STATUS_A_ZADA          (1U << 2)
#define LSM303D_STATUS_A_YADA          (1U << 1)
#define LSM303D_STATUS_A_XADA          (1U << 0)

/* --- STATUS_M (0x07) --- */
#define LSM303D_STATUS_M_ZYXOW         (1U << 7)
#define LSM303D_STATUS_M_ZOW           (1U << 6)
#define LSM303D_STATUS_M_YOW           (1U << 5)
#define LSM303D_STATUS_M_XOW           (1U << 4)
#define LSM303D_STATUS_M_ZYXDA         (1U << 3)  /**< Mag new data ready for X, Y, Z */
#define LSM303D_STATUS_M_ZDA           (1U << 2)
#define LSM303D_STATUS_M_YDA           (1U << 1)
#define LSM303D_STATUS_M_XDA           (1U << 0)

/*============================================================================*
 *                   NOMINAL SENSITIVITIES & SCALE FACTORS                    *
 *============================================================================*/

/**
 * @name Accelerometer sensitivities (LSB to g conversion)
 * From LSM303D Datasheet Table 3:
 *   ±2g:   0.061 mg/LSB = 0.000061 g/LSB
 *   ±4g:   0.122 mg/LSB = 0.000122 g/LSB
 *   ±6g:   0.183 mg/LSB = 0.000183 g/LSB
 *   ±8g:   0.244 mg/LSB = 0.000244 g/LSB
 *   ±16g:  0.732 mg/LSB = 0.000732 g/LSB
 * @{
 */
#define LSM303D_ACCEL_SENS_2G          0.000061f
#define LSM303D_ACCEL_SENS_4G          0.000122f
#define LSM303D_ACCEL_SENS_6G          0.000183f
#define LSM303D_ACCEL_SENS_8G          0.000244f
#define LSM303D_ACCEL_SENS_16G         0.000732f
/** @} */

/**
 * @name Magnetometer sensitivities (LSB to Gauss conversion)
 * From LSM303D Datasheet Table 3:
 *   ±2 Gauss:   0.080 mgauss/LSB = 0.000080 Gauss/LSB (0.0080 µT/LSB)
 *   ±4 Gauss:   0.160 mgauss/LSB = 0.000160 Gauss/LSB (0.0160 µT/LSB)
 *   ±8 Gauss:   0.320 mgauss/LSB = 0.000320 Gauss/LSB (0.0320 µT/LSB)
 *   ±12 Gauss:  0.479 mgauss/LSB = 0.000479 Gauss/LSB (0.0479 µT/LSB)
 * @{
 */
#define LSM303D_MAG_SENS_2GAUSS        0.000080f
#define LSM303D_MAG_SENS_4GAUSS        0.000160f
#define LSM303D_MAG_SENS_8GAUSS        0.000320f
#define LSM303D_MAG_SENS_12GAUSS       0.000479f
/** @} */

/** @brief Standard earth gravity [m/s²]. */
#ifndef GRAVITY_EARTH_MPS2
#define GRAVITY_EARTH_MPS2             9.80665f
#endif

/** @brief Conversion factor from Gauss to Microtesla (1 Gauss = 100 µT). */
#define GAUSS_TO_MICROTESLA            100.0f

/** @brief Embedded temperature sensor sensitivity: 8 LSB / °C => 0.125 °C / LSB. */
#define LSM303D_TEMP_SENS_DEG_C        0.125f

/** @brief Baseline reference temperature in °C corresponding to 0 LSB offset. */
#define LSM303D_TEMP_OFFSET_DEG_C      25.0f

#ifdef __cplusplus
}
#endif

#endif /* LSM303D_PRIVATE_H */
