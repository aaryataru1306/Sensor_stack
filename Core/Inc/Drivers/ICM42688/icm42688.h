
/**
 ******************************************************************************
 * @file    icm42688.h
 * @brief   Public API for the TDK InvenSense ICM-42688-P 6-axis High-Performance
 *          MotionTracking MEMS IMU (Accelerometer + Gyroscope + Temperature).
 * @details
 *   Key Sensor Characteristics:
 *     - Model: TDK InvenSense ICM-42688-P (6-Axis MEMS MotionTracking)
 *     - Gyro Range: Programmable ±15.625°/s up to ±2000°/s
 *     - Gyro Noise Density: 2.8 mdps / √Hz (Ultra-low noise floor)
 *     - Accel Range: Programmable ±2 g up to ±16 g
 *     - Interface / Rate: SPI (up to 24 MHz) / I2C | Output Data Rate up to 32 kHz
 *     - Role: High-precision attitude estimation for GPS-denied optical flow & LiDAR
 *
 *   Bus Interface:
 *     Hardware-agnostic driver. Built on top of the shared bare-metal SPI1
 *     driver (Protocols/SPI/spi_driver.h). Chip-select pin defaults to PA1
 *     (SPI1_CS_ICM42688_PIN / PIN_ICM42688_CS).
 *
 *   Zero HAL_* calls, zero dynamic allocation, strictly bare-metal.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            CONFIGURATION TYPES                             *
 *============================================================================*/

/**
 * @brief Gyroscope full-scale range (GYRO_CONFIG0 bits[7:5]).
 *        Supports ranges from ±15.625 dps up to ±2000 dps.
 */
typedef enum {
    ICM42688_GYRO_FS_2000DPS    = 0x00, /**< ±2000 °/s (16.4 LSB/dps) */
    ICM42688_GYRO_FS_1000DPS    = 0x01, /**< ±1000 °/s (32.8 LSB/dps) */
    ICM42688_GYRO_FS_500DPS     = 0x02, /**< ±500 °/s  (65.5 LSB/dps) */
    ICM42688_GYRO_FS_250DPS     = 0x03, /**< ±250 °/s  (131.0 LSB/dps) */
    ICM42688_GYRO_FS_125DPS     = 0x04, /**< ±125 °/s  (262.0 LSB/dps) */
    ICM42688_GYRO_FS_62_5DPS    = 0x05, /**< ±62.5 °/s (524.3 LSB/dps) */
    ICM42688_GYRO_FS_31_25DPS   = 0x06, /**< ±31.25 °/s (1048.6 LSB/dps) */
    ICM42688_GYRO_FS_15_625DPS  = 0x07  /**< ±15.625 °/s (2097.2 LSB/dps) */
} ICM42688_GyroFS_t;

/**
 * @brief Accelerometer full-scale range (ACCEL_CONFIG0 bits[7:5]).
 */
typedef enum {
    ICM42688_ACCEL_FS_16G = 0x00, /**< ±16 g (2,048 LSB/g) */
    ICM42688_ACCEL_FS_8G  = 0x01, /**< ±8 g  (4,096 LSB/g) */
    ICM42688_ACCEL_FS_4G  = 0x02, /**< ±4 g  (8,192 LSB/g) */
    ICM42688_ACCEL_FS_2G  = 0x03  /**< ±2 g  (16,384 LSB/g) */
} ICM42688_AccelFS_t;

/**
 * @brief Output Data Rate (ODR) for Gyroscope and Accelerometer.
 *        Supports up to 32 kHz.
 */
typedef enum {
    ICM42688_ODR_32KHZ   = 0x01, /**< 32 kHz */
    ICM42688_ODR_16KHZ   = 0x02, /**< 16 kHz */
    ICM42688_ODR_8KHZ    = 0x03, /**< 8 kHz */
    ICM42688_ODR_4KHZ    = 0x04, /**< 4 kHz */
    ICM42688_ODR_2KHZ    = 0x05, /**< 2 kHz */
    ICM42688_ODR_1KHZ    = 0x06, /**< 1 kHz (standard high-rate) */
    ICM42688_ODR_200HZ   = 0x07, /**< 200 Hz */
    ICM42688_ODR_100HZ   = 0x08, /**< 100 Hz (standard EKF fusion rate) */
    ICM42688_ODR_50HZ    = 0x09, /**< 50 Hz */
    ICM42688_ODR_25HZ    = 0x0A, /**< 25 Hz */
    ICM42688_ODR_12_5HZ  = 0x0B, /**< 12.5 Hz */
    ICM42688_ODR_500HZ   = 0x0F  /**< 500 Hz */
} ICM42688_ODR_t;

/**
 * @brief Digital Low-Pass Filter (DLPF) / UI Filter Bandwidth.
 */
typedef enum {
    ICM42688_FILTER_BW_ODR_DIV_2  = 0x00, /**< BW = ODR / 2 (Low latency) */
    ICM42688_FILTER_BW_ODR_DIV_4  = 0x01, /**< BW = max(400Hz, ODR) / 4 */
    ICM42688_FILTER_BW_ODR_DIV_5  = 0x02, /**< BW = max(400Hz, ODR) / 5 */
    ICM42688_FILTER_BW_ODR_DIV_8  = 0x03, /**< BW = max(400Hz, ODR) / 8 */
    ICM42688_FILTER_BW_ODR_DIV_10 = 0x04, /**< BW = max(400Hz, ODR) / 10 */
    ICM42688_FILTER_BW_ODR_DIV_16 = 0x05, /**< BW = max(400Hz, ODR) / 16 */
    ICM42688_FILTER_BW_ODR_DIV_20 = 0x06, /**< BW = max(400Hz, ODR) / 20 */
    ICM42688_FILTER_BW_ODR_DIV_40 = 0x07, /**< BW = max(400Hz, ODR) / 40 */
    ICM42688_FILTER_BYPASS        = 0x0F  /**< Filter bypassed (ultra-low latency) */
} ICM42688_FilterBW_t;

/**
 * @brief Driver status return codes.
 */
typedef enum {
    ICM42688_OK = 0,               /**< Operation succeeded. */
    ICM42688_ERROR_WHOAMI,         /**< WHO_AM_I mismatch (expected 0x47). */
    ICM42688_ERROR_INVALID_PARAM,  /**< Invalid argument or null pointer. */
    ICM42688_ERROR_BUS,            /**< SPI bus communication failure. */
    ICM42688_ERROR_NOT_INITIALIZED /**< Called before ICM42688_Init(). */
} ICM42688_Status_t;

/**
 * @brief Consolidated 6-axis IMU sample + die temperature.
 * @details Acceleration in g, angular velocity in °/s and rad/s, temp in °C.
 */
typedef struct {
    float accel_x;       /**< Acceleration X-axis [g] (1 g ≈ 9.80665 m/s²). */
    float accel_y;       /**< Acceleration Y-axis [g]. */
    float accel_z;       /**< Acceleration Z-axis [g]. */

    float gyro_x;        /**< Angular rate X-axis [°/s]. */
    float gyro_y;        /**< Angular rate Y-axis [°/s]. */
    float gyro_z;        /**< Angular rate Z-axis [°/s]. */

    float gyro_rad_x;    /**< Angular rate X-axis [rad/s] (ready for EKF). */
    float gyro_rad_y;    /**< Angular rate Y-axis [rad/s]. */
    float gyro_rad_z;    /**< Angular rate Z-axis [rad/s]. */

    float temp_c;        /**< Die temperature [°C]. */
} ICM42688_Data_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Brings up the ICM-42688-P: performs soft reset, verifies WHO_AM_I (0x47),
 *        enables internal Gyro PLL, puts both Accel and Gyro into Low-Noise (LN) mode
 *        (enabling the 2.8 mdps/√Hz noise density), and configures full-scale and ODR.
 *
 * @param[in] accel_fs   Accelerometer full-scale range (±2g to ±16g).
 * @param[in] gyro_fs    Gyroscope full-scale range (±15.625 dps to ±2000 dps).
 * @param[in] accel_odr  Accelerometer output data rate.
 * @param[in] gyro_odr   Gyroscope output data rate.
 *
 * @retval ICM42688_OK            Device successfully initialized.
 * @retval ICM42688_ERROR_WHOAMI  WHO_AM_I register mismatch.
 * @retval ICM42688_ERROR_BUS     SPI communication error.
 */
ICM42688_Status_t ICM42688_Init(ICM42688_AccelFS_t accel_fs,
                                ICM42688_GyroFS_t  gyro_fs,
                                ICM42688_ODR_t     accel_odr,
                                ICM42688_ODR_t     gyro_odr);

/**
 * @brief Default bring-up tailored for high-precision UAV / Optical Flow attitude estimation:
 *        ±2000 dps gyro, ±16 g accel, 1 kHz ODR, Low-Noise Mode, DLPF enabled.
 *
 * @retval ICM42688_OK on success.
 */
ICM42688_Status_t ICM42688_InitDefault(void);

/**
 * @brief Reads the WHO_AM_I register (0x75, Bank 0).
 * @return 0x47 on a genuine ICM-42688-P; 0x00 on bus/wiring fault.
 */
uint8_t ICM42688_WhoAmI(void);

/**
 * @brief Reads a coherent 14-byte burst containing Temperature, Accelerometer,
 *        and Gyroscope data, converting raw LSBs into physical engineering units.
 *
 * @param[out] data  Destination structure. Must not be NULL.
 * @retval ICM42688_OK on success.
 */
ICM42688_Status_t ICM42688_ReadAll(ICM42688_Data_t *data);

/**
 * @brief Reads only the 3-axis Accelerometer data (converted to g).
 * @param[out] ax, ay, az  Pointers to receive acceleration [g].
 */
ICM42688_Status_t ICM42688_ReadAccel(float *ax, float *ay, float *az);

/**
 * @brief Reads only the 3-axis Gyroscope data (converted to °/s).
 * @param[out] gx, gy, gz  Pointers to receive angular rates [°/s].
 */
ICM42688_Status_t ICM42688_ReadGyro(float *gx, float *gy, float *gz);

/**
 * @brief Reads the die temperature in degrees Celsius.
 * @param[out] temp_c  Pointer to receive temperature [°C].
 */
ICM42688_Status_t ICM42688_ReadTemp(float *temp_c);

/**
 * @brief Configures digital filter bandwidth for anti-aliasing / vibration rejection.
 * @param[in] gyro_bw   Gyroscope filter bandwidth.
 * @param[in] accel_bw  Accelerometer filter bandwidth.
 */
ICM42688_Status_t ICM42688_SetFilters(ICM42688_FilterBW_t gyro_bw,
                                      ICM42688_FilterBW_t accel_bw);

/**
 * @brief Checks if fresh data is ready in the sensor data registers.
 * @retval true if DATA_RDY flag is asserted in INT_STATUS (0x2D).
 */
bool ICM42688_IsDataReady(void);

/**
 * @brief Triggers a software reset of the ICM-42688-P via DEVICE_CONFIG.
 */
ICM42688_Status_t ICM42688_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
