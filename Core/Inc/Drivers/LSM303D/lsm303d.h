/**
 ******************************************************************************
 * @file    lsm303d.h
 * @brief   Public API for the STMicroelectronics LSM303D 6-axis eCompass module
 *          (3D Accelerometer + 3D Magnetometer + Temperature Sensor).
 * @details
 *   Sensor Characteristics & Supported Protocols:
 *     - Accelerometer Dynamic Range: ±2 g, ±4 g, ±6 g, ±8 g, ±16 g
 *     - Accelerometer ODR: 3.125 Hz up to 1600 Hz
 *     - Magnetometer Dynamic Range: ±2, ±4, ±8, ±12 Gauss
 *     - Magnetometer ODR: 3.125 Hz up to 100 Hz
 *     - Integrated 12-bit Temperature Sensor (0.125 °C/LSB)
 *
 *   Bus Communication Support:
 *     The LSM303D silicon natively supports two digital serial interfaces:
 *       1. I2C Interface:
 *          - Standard (100 kHz) and Fast (400 kHz) mode
 *          - 7-bit slave address: 0x1E (SDO/SA0 to GND) or 0x1D (SDO/SA0 to VDD)
 *          - Multi-byte continuous reading with sub-address MSB auto-increment
 *       2. SPI Interface:
 *          - Standard 4-wire (and 3-wire) SPI modes up to 10 MHz
 *          - SPI Modes 0 and 3
 *          - Active-low CS framing with bit 7 (R/W) and bit 6 (MS auto-increment)
 *
 *     This driver supports BOTH buses transparently via @ref LSM303D_BusConfig_t,
 *     integrating seamlessly with the bare-metal I2C (i2c_interface.h) and
 *     SPI1 (spi_driver.h) protocol layers.
 *
 *   Zero HAL_* dependencies, zero dynamic memory allocation, strictly MISRA-compliant.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef LSM303D_H
#define LSM303D_H

#include <stdint.h>
#include <stdbool.h>
#include "ErrTypes.h"
#include "i2c_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            CONFIGURATION TYPES                             *
 *============================================================================*/

/**
 * @brief Communication protocol bus selector.
 */
typedef enum {
    LSM303D_BUS_SPI = 0, /**< SPI bus (e.g. shared SPI1 with CS pin). */
    LSM303D_BUS_I2C = 1  /**< I2C bus (e.g. I2C1 with 7-bit slave address). */
} LSM303D_BusType_t;

/**
 * @brief Bus configuration struct passed during driver initialization.
 */
typedef struct {
    LSM303D_BusType_t bus_type;   /**< Selected communication protocol (SPI or I2C). */
    uint8_t           spi_cs_pin; /**< GPIOA pin number for SPI CS (used if bus_type == LSM303D_BUS_SPI). */
    I2C_Config_t     *i2c_cfg;    /**< Pointer to I2C peripheral configuration (used if bus_type == LSM303D_BUS_I2C). */
    uint8_t           i2c_addr;   /**< 7-bit I2C slave address: 0x1E (GND) or 0x1D (VDD). */
} LSM303D_BusConfig_t;

/**
 * @brief Accelerometer full-scale range (CTRL_REG2 AFS[2:0]).
 */
typedef enum {
    LSM303D_ACCEL_FS_2G  = 0x00, /**< ±2 g  (0.061 mg/LSB) */
    LSM303D_ACCEL_FS_4G  = 0x01, /**< ±4 g  (0.122 mg/LSB) */
    LSM303D_ACCEL_FS_6G  = 0x02, /**< ±6 g  (0.183 mg/LSB) */
    LSM303D_ACCEL_FS_8G  = 0x03, /**< ±8 g  (0.244 mg/LSB) */
    LSM303D_ACCEL_FS_16G = 0x04  /**< ±16 g (0.732 mg/LSB) */
} LSM303D_AccelFS_t;

/**
 * @brief Accelerometer Output Data Rate (CTRL_REG1 AODR[3:0]).
 */
typedef enum {
    LSM303D_ACCEL_ODR_POWERDOWN = 0x00, /**< Power-down mode */
    LSM303D_ACCEL_ODR_3_125HZ   = 0x01, /**< 3.125 Hz */
    LSM303D_ACCEL_ODR_6_25HZ    = 0x02, /**< 6.25 Hz */
    LSM303D_ACCEL_ODR_12_5HZ    = 0x03, /**< 12.5 Hz */
    LSM303D_ACCEL_ODR_25HZ      = 0x04, /**< 25 Hz */
    LSM303D_ACCEL_ODR_50HZ      = 0x05, /**< 50 Hz */
    LSM303D_ACCEL_ODR_100HZ     = 0x06, /**< 100 Hz (standard fusion rate) */
    LSM303D_ACCEL_ODR_200HZ     = 0x07, /**< 200 Hz */
    LSM303D_ACCEL_ODR_400HZ     = 0x08, /**< 400 Hz */
    LSM303D_ACCEL_ODR_800HZ     = 0x09, /**< 800 Hz */
    LSM303D_ACCEL_ODR_1600HZ    = 0x0A  /**< 1600 Hz */
} LSM303D_AccelODR_t;

/**
 * @brief Accelerometer anti-alias filter bandwidth (CTRL_REG2 ABW[1:0]).
 */
typedef enum {
    LSM303D_ACCEL_BW_773HZ = 0x00, /**< 773 Hz */
    LSM303D_ACCEL_BW_194HZ = 0x01, /**< 194 Hz */
    LSM303D_ACCEL_BW_362HZ = 0x02, /**< 362 Hz */
    LSM303D_ACCEL_BW_50HZ  = 0x03  /**< 50 Hz (high noise rejection) */
} LSM303D_AccelBW_t;

/**
 * @brief Magnetometer full-scale range (CTRL_REG6 MFS[1:0]).
 */
typedef enum {
    LSM303D_MAG_FS_2GAUSS  = 0x00, /**< ±2 Gauss  (0.080 mGauss/LSB = 0.0080 µT/LSB) */
    LSM303D_MAG_FS_4GAUSS  = 0x01, /**< ±4 Gauss  (0.160 mGauss/LSB = 0.0160 µT/LSB) */
    LSM303D_MAG_FS_8GAUSS  = 0x02, /**< ±8 Gauss  (0.320 mGauss/LSB = 0.0320 µT/LSB) */
    LSM303D_MAG_FS_12GAUSS = 0x03  /**< ±12 Gauss (0.479 mGauss/LSB = 0.0479 µT/LSB) */
} LSM303D_MagFS_t;

/**
 * @brief Magnetometer Output Data Rate (CTRL_REG5 M_ODR[2:0]).
 */
typedef enum {
    LSM303D_MAG_ODR_3_125HZ = 0x00, /**< 3.125 Hz */
    LSM303D_MAG_ODR_6_25HZ  = 0x01, /**< 6.25 Hz */
    LSM303D_MAG_ODR_12_5HZ  = 0x02, /**< 12.5 Hz */
    LSM303D_MAG_ODR_25HZ    = 0x03, /**< 25 Hz */
    LSM303D_MAG_ODR_50HZ    = 0x04, /**< 50 Hz */
    LSM303D_MAG_ODR_100HZ   = 0x05  /**< 100 Hz (fastest magnetometer rate) */
} LSM303D_MagODR_t;

/**
 * @brief Driver status return codes.
 */
typedef enum {
    LSM303D_OK = 0,                /**< Operation succeeded. */
    LSM303D_ERROR_WHOAMI,          /**< WHO_AM_I check failed (expected 0x49). */
    LSM303D_ERROR_INVALID_PARAM,   /**< Invalid argument or null pointer. */
    LSM303D_ERROR_BUS,             /**< Bus communication error (I2C NACK or SPI fault). */
    LSM303D_ERROR_NOT_INITIALIZED  /**< Driver called before successful initialization. */
} LSM303D_Status_t;

/**
 * @brief Consolidated sample container holding calibrated data for all channels.
 */
typedef struct {
    /* Accelerometer data */
    float accel_x;       /**< Linear acceleration X-axis [g] (1 g ≈ 9.80665 m/s²). */
    float accel_y;       /**< Linear acceleration Y-axis [g]. */
    float accel_z;       /**< Linear acceleration Z-axis [g]. */
    float accel_mps2_x;  /**< Linear acceleration X-axis [m/s²]. */
    float accel_mps2_y;  /**< Linear acceleration Y-axis [m/s²]. */
    float accel_mps2_z;  /**< Linear acceleration Z-axis [m/s²]. */

    /* Magnetometer data */
    float mag_x;         /**< Magnetic flux density X-axis [Gauss]. */
    float mag_y;         /**< Magnetic flux density Y-axis [Gauss]. */
    float mag_z;         /**< Magnetic flux density Z-axis [Gauss]. */
    float mag_uT_x;      /**< Magnetic flux density X-axis [µT] (1 Gauss = 100 µT). */
    float mag_uT_y;      /**< Magnetic flux density Y-axis [µT]. */
    float mag_uT_z;      /**< Magnetic flux density Z-axis [µT]. */

    /* Temperature */
    float temp_c;        /**< Die temperature [°C]. */
} LSM303D_Data_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Generic bring-up for the LSM303D sensor across any supported bus.
 *
 * @details
 *   Initialization sequence:
 *     1. Verify communication and device identity via WHO_AM_I (0x0F == 0x49).
 *     2. Perform software reboot to reset registers to a known clean state.
 *     3. Enable Block Data Update (BDU = 1) to prevent reading torn words.
 *     4. Enable all three accelerometer axes (X, Y, Z) and configure ODR.
 *     5. Configure accelerometer anti-alias filter bandwidth and full-scale range.
 *     6. Enable internal temperature sensor and configure magnetometer resolution (high-res) & ODR.
 *     7. Configure magnetometer full-scale range.
 *     8. Put magnetometer into continuous-conversion mode (MD = 00b).
 *
 * @param[in] bus_cfg    Bus configuration structure (must not be NULL).
 * @param[in] accel_fs   Accelerometer dynamic measurement range.
 * @param[in] accel_odr  Accelerometer output data rate.
 * @param[in] mag_fs     Magnetometer dynamic measurement range.
 * @param[in] mag_odr    Magnetometer output data rate.
 *
 * @retval LSM303D_OK                   Initialization succeeded.
 * @retval LSM303D_ERROR_WHOAMI         Device returned unexpected WHO_AM_I.
 * @retval LSM303D_ERROR_INVALID_PARAM  Null pointer or invalid parameter passed.
 * @retval LSM303D_ERROR_BUS            Bus communication failed.
 */
LSM303D_Status_t LSM303D_Init(const LSM303D_BusConfig_t *bus_cfg,
                              LSM303D_AccelFS_t          accel_fs,
                              LSM303D_AccelODR_t         accel_odr,
                              LSM303D_MagFS_t            mag_fs,
                              LSM303D_MagODR_t           mag_odr);

/**
 * @brief Convenience initializer for SPI bus operation with sensible flight/motion defaults:
 *        ±2g accel, 100 Hz ODR, 50 Hz filter BW, ±4 Gauss mag, 100 Hz mag ODR, temp enabled.
 *
 * @param[in] cs_pin  GPIOA pin number for SPI CS (e.g. 0 for PA0).
 */
LSM303D_Status_t LSM303D_InitDefaultSPI(uint8_t cs_pin);

/**
 * @brief Convenience initializer for I2C bus operation with sensible flight/motion defaults:
 *        ±2g accel, 100 Hz ODR, 50 Hz filter BW, ±4 Gauss mag, 100 Hz mag ODR, temp enabled.
 *
 * @param[in] i2c_cfg   Initialized bare-metal I2C configuration.
 * @param[in] i2c_addr  7-bit I2C slave address (0x1E or 0x1D).
 */
LSM303D_Status_t LSM303D_InitDefaultI2C(I2C_Config_t *i2c_cfg, uint8_t i2c_addr);

/**
 * @brief Read WHO_AM_I device identification register (0x0F).
 * @return Register value: 0x49 on genuine LSM303D; 0x00 on bus/wiring fault.
 */
uint8_t LSM303D_WhoAmI(void);

/**
 * @brief Perform software reboot of device memory contents via CTRL_REG0 BOOT bit.
 */
LSM303D_Status_t LSM303D_SoftReset(void);

/**
 * @brief Read 3-axis accelerometer data converted to physical units [g].
 *
 * @param[out] ax, ay, az  Pointers to receive linear acceleration in g.
 */
LSM303D_Status_t LSM303D_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief Read 3-axis accelerometer data converted to SI units [m/s²].
 *
 * @param[out] ax, ay, az  Pointers to receive linear acceleration in m/s².
 */
LSM303D_Status_t LSM303D_ReadAccelMps2(float *ax, float *ay, float *az);

/**
 * @brief Read raw 16-bit two's complement accelerometer register counts.
 *
 * @param[out] rx, ry, rz  Pointers to receive raw counts.
 */
LSM303D_Status_t LSM303D_ReadAccelRaw(int16_t *rx, int16_t *ry, int16_t *rz);

/**
 * @brief Read 3-axis magnetometer data converted to physical units [Gauss].
 *
 * @param[out] mx, my, mz  Pointers to receive magnetic field in Gauss.
 */
LSM303D_Status_t LSM303D_ReadMag(float *mx, float *my, float *mz);

/**
 * @brief Read 3-axis magnetometer data converted to Microteslas [µT].
 *
 * @param[out] mx, my, mz  Pointers to receive magnetic field in µT (1 Gauss = 100 µT).
 */
LSM303D_Status_t LSM303D_ReadMagMicroTesla(float *mx, float *my, float *mz);

/**
 * @brief Read raw 16-bit two's complement magnetometer register counts.
 *
 * @param[out] rx, ry, rz  Pointers to receive raw counts.
 */
LSM303D_Status_t LSM303D_ReadMagRaw(int16_t *rx, int16_t *ry, int16_t *rz);

/**
 * @brief Read internal die temperature sensor in degrees Celsius [°C].
 *
 * @param[out] temp_c  Pointer to receive temperature in °C.
 */
LSM303D_Status_t LSM303D_ReadTemp(float *temp_c);

/**
 * @brief Coherent burst read of all sensor channels (Accel, Mag, Temp).
 *
 * @param[out] data  Pointer to destination structure. Must not be NULL.
 */
LSM303D_Status_t LSM303D_ReadAll(LSM303D_Data_t *data);

/**
 * @brief Check if fresh accelerometer data is available.
 * @retval true if STATUS_A ZYXADA bit is set.
 */
bool LSM303D_IsAccelDataReady(void);

/**
 * @brief Check if fresh magnetometer data is available.
 * @retval true if STATUS_M ZYXDA bit is set.
 */
bool LSM303D_IsMagDataReady(void);

/**
 * @brief Update accelerometer anti-alias filter bandwidth dynamically.
 *
 * @param[in] bw  Filter bandwidth selection.
 */
LSM303D_Status_t LSM303D_SetAccelFilterBW(LSM303D_AccelBW_t bw);

/**
 * @brief Perform still-start offset calibration for the accelerometer.
 *
 * @details
 *   Averages @p samples readings with the vehicle stationary, subtracting
 *   the expected 1 g vector from the vertical (Z) axis.
 *
 * @param[in] samples  Number of samples to average (e.g. 100 to 400).
 */
LSM303D_Status_t LSM303D_CalibrateAccelOffsets(uint16_t samples);

#ifdef __cplusplus
}
#endif

#endif /* LSM303D_H */
