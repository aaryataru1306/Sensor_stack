/**
 ******************************************************************************
 * @file    icm42688.c
 * @brief   Implementation of the TDK InvenSense ICM-42688-P 6-axis IMU driver.
 * @details
 *   Pipeline: ICM42688_ReadAll() -> 14-byte SPI burst (Temp, Accel, Gyro)
 *             -> conversion using configured full-scale sensitivities
 *             -> caller-supplied ICM42688_Data_t.
 *
 *   Bus Interface:
 *     Transfers use the register-level bare-metal SPI1 driver (spi_driver.h).
 *     Chip-select is on PA1 (SPI1_CS_ICM42688_PIN).
 *
 *   Noise Performance:
 *     Initializes both Gyroscope and Accelerometer in Low-Noise (LN) mode
 *     (PWR_MGMT0 = 0x0F), providing the specified 2.8 mdps/√Hz ultra-low
 *     noise floor essential for GPS-denied optical flow & LiDAR attitude fusion.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#include "icm42688.h"
#include "icm42688_private.h"
#include "spi_driver.h"
#include <string.h>

#ifndef SPI1_CS_ICM42688_PIN
#define SPI1_CS_ICM42688_PIN 1U
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD (M_PI / 180.0f)

/*============================================================================*
 *                          INTERNAL DRIVER STATE                             *
 *============================================================================*/

static uint8_t s_currentBank = 0xFFU;  /**< Cached bank to eliminate redundant writes */
static float   s_accelSens   = ICM42688_ACCEL_SENS_16G;  /**< LSB / g */
static float   s_gyroSens    = ICM42688_GYRO_SENS_2000DPS; /**< LSB / (°/s) */
static bool    s_initialized = false;

/*============================================================================*
 *                          INTERNAL BUS PRIMITIVES                           *
 *============================================================================*/

static void icm42688_delay_ms(uint32_t ms)
{
    /* Spin loop calibrated for STM32F411 at 100 MHz SYSCLK */
    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 20000U; j++) {
            __asm__("nop");
        }
    }
}

static void ICM42688_WriteReg(uint8_t reg, uint8_t value)
{
    /* Bit 7 = 0 for SPI write */
    SPI1_WriteRegister(SPI1_CS_ICM42688_PIN, (uint8_t)(reg & 0x7FU), value);
}

static void ICM42688_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* Bit 7 = 1 for SPI read */
    SPI1_ReadRegisters(SPI1_CS_ICM42688_PIN, (uint8_t)(reg | 0x80U), buf, len);
}

static void ICM42688_SelectBank(uint8_t bank)
{
    if (bank == s_currentBank) {
        return;
    }
    ICM42688_WriteReg(ICM42688_REG_BANK_SEL, (uint8_t)(bank & 0x07U));
    s_currentBank = bank;
}

static void ICM42688_WriteRegBank(uint8_t bank, uint8_t reg, uint8_t value)
{
    ICM42688_SelectBank(bank);
    ICM42688_WriteReg(reg, value);
}

static void ICM42688_ReadRegsBank(uint8_t bank, uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM42688_SelectBank(bank);
    ICM42688_ReadRegs(reg, buf, len);
}

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

uint8_t ICM42688_WhoAmI(void)
{
    uint8_t who = 0U;
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_WHO_AM_I, &who, 1U);
    return who;
}

ICM42688_Status_t ICM42688_Reset(void)
{
    /* Trigger device reset via DEVICE_CONFIG bit 0 */
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_DEVICE_CONFIG, ICM42688_SOFT_RESET_BIT);
    icm42688_delay_ms(10U);
    s_currentBank = 0xFFU;
    s_initialized = false;
    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_Init(ICM42688_AccelFS_t accel_fs,
                                ICM42688_GyroFS_t  gyro_fs,
                                ICM42688_ODR_t     accel_odr,
                                ICM42688_ODR_t     gyro_odr)
{
    /* 1. Software Reset */
    (void)ICM42688_Reset();

    /* 2. Check Device Identity */
    uint8_t who = ICM42688_WhoAmI();
    if (who != ICM42688_WHO_AM_I_VALUE) {
        return ICM42688_ERROR_WHOAMI;
    }

    /* 3. Interface Config: Disable I2C interface to protect against SPI glitches */
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_INTF_CONFIG0,
                          ICM42688_UI_SIFS_DISABLE_I2C);

    /* 4. Clock selection: Auto-select PLL when available for maximum stability */
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_INTF_CONFIG1,
                          ICM42688_CLKSEL_PLL_OR_RC);

    /* 5. Power Management: Turn on Gyroscope and Accelerometer in Low-Noise (LN) Mode
     *    and keep Temperature Sensor enabled.
     *    Low-Noise Mode enables the 2.8 mdps/√Hz gyro noise density. */
    uint8_t pwr = ICM42688_PWR_TEMP_ON |
                  ICM42688_GYRO_MODE_LOW_NOISE |
                  ICM42688_ACCEL_MODE_LOW_NOISE;
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_PWR_MGMT0, pwr);
    icm42688_delay_ms(5U); /* Allow Gyroscope PLL to lock and MEMS to stabilize */

    /* 6. Configure Gyroscope: Full-Scale Range and Output Data Rate */
    uint8_t gyro_cfg0 = (uint8_t)((((uint8_t)gyro_fs & 0x07U) << 5U) |
                                  ((uint8_t)gyro_odr & 0x0FU));
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_GYRO_CONFIG0, gyro_cfg0);

    /* 7. Configure Accelerometer: Full-Scale Range and Output Data Rate */
    uint8_t accel_cfg0 = (uint8_t)((((uint8_t)accel_fs & 0x03U) << 5U) |
                                   ((uint8_t)accel_odr & 0x0FU));
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_ACCEL_CONFIG0, accel_cfg0);

    /* 8. Configure Digital Low-Pass Filter (DLPF):
     *    Set Gyro and Accel UI filter bandwidth to ODR/4 (balanced latency & vibration rejection) */
    uint8_t filt_cfg = (uint8_t)((ICM42688_FILTER_BW_ODR_DIV_4 << 4U) |
                                 ICM42688_FILTER_BW_ODR_DIV_4);
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_GYRO_ACCEL_CONFIG0, filt_cfg);

    /* 9. Cache sensitivity scale factors according to selected range */
    switch (accel_fs) {
        case ICM42688_ACCEL_FS_16G: s_accelSens = ICM42688_ACCEL_SENS_16G; break;
        case ICM42688_ACCEL_FS_8G:  s_accelSens = ICM42688_ACCEL_SENS_8G;  break;
        case ICM42688_ACCEL_FS_4G:  s_accelSens = ICM42688_ACCEL_SENS_4G;  break;
        case ICM42688_ACCEL_FS_2G:  s_accelSens = ICM42688_ACCEL_SENS_2G;  break;
        default:                    s_accelSens = ICM42688_ACCEL_SENS_16G; break;
    }

    switch (gyro_fs) {
        case ICM42688_GYRO_FS_2000DPS:   s_gyroSens = ICM42688_GYRO_SENS_2000DPS;   break;
        case ICM42688_GYRO_FS_1000DPS:   s_gyroSens = ICM42688_GYRO_SENS_1000DPS;   break;
        case ICM42688_GYRO_FS_500DPS:    s_gyroSens = ICM42688_GYRO_SENS_500DPS;    break;
        case ICM42688_GYRO_FS_250DPS:    s_gyroSens = ICM42688_GYRO_SENS_250DPS;    break;
        case ICM42688_GYRO_FS_125DPS:    s_gyroSens = ICM42688_GYRO_SENS_125DPS;    break;
        case ICM42688_GYRO_FS_62_5DPS:   s_gyroSens = ICM42688_GYRO_SENS_62_5DPS;   break;
        case ICM42688_GYRO_FS_31_25DPS:  s_gyroSens = ICM42688_GYRO_SENS_31_25DPS;  break;
        case ICM42688_GYRO_FS_15_625DPS: s_gyroSens = ICM42688_GYRO_SENS_15_625DPS; break;
        default:                         s_gyroSens = ICM42688_GYRO_SENS_2000DPS;   break;
    }

    /* 10. Return to resting Bank 0 */
    ICM42688_SelectBank(ICM42688_BANK_0);

    s_initialized = true;
    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_InitDefault(void)
{
    /* High-Performance configuration for UAVs & Optical Flow attitude estimation:
     * ±16 g accel, ±2000 dps gyro, 1 kHz ODR, Low-Noise mode */
    return ICM42688_Init(ICM42688_ACCEL_FS_16G,
                         ICM42688_GYRO_FS_2000DPS,
                         ICM42688_ODR_1KHZ,
                         ICM42688_ODR_1KHZ);
}

ICM42688_Status_t ICM42688_ReadAll(ICM42688_Data_t *data)
{
    if (data == NULL) {
        return ICM42688_ERROR_INVALID_PARAM;
    }

    /* Coherent 14-byte burst read from 0x1D to 0x2A:
     * [0..1]   = TEMP_DATA1 (MSB), TEMP_DATA0 (LSB)
     * [2..7]   = ACCEL_X (H, L), ACCEL_Y (H, L), ACCEL_Z (H, L)
     * [8..13]  = GYRO_X  (H, L), GYRO_Y  (H, L), GYRO_Z  (H, L) */
    uint8_t raw[14];
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_TEMP_DATA1, raw, 14U);

    int16_t raw_temp = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    int16_t raw_ax   = (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);
    int16_t raw_ay   = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);
    int16_t raw_az   = (int16_t)(((uint16_t)raw[6] << 8U) | (uint16_t)raw[7]);
    int16_t raw_gx   = (int16_t)(((uint16_t)raw[8] << 8U) | (uint16_t)raw[9]);
    int16_t raw_gy   = (int16_t)(((uint16_t)raw[10] << 8U) | (uint16_t)raw[11]);
    int16_t raw_gz   = (int16_t)(((uint16_t)raw[12] << 8U) | (uint16_t)raw[13]);

    /* Physical conversions */
    data->temp_c  = ((float)raw_temp / ICM42688_TEMP_SENSITIVITY) + ICM42688_TEMP_OFFSET_C;

    data->accel_x = (float)raw_ax / s_accelSens;
    data->accel_y = (float)raw_ay / s_accelSens;
    data->accel_z = (float)raw_az / s_accelSens;

    data->gyro_x  = (float)raw_gx / s_gyroSens;
    data->gyro_y  = (float)raw_gy / s_gyroSens;
    data->gyro_z  = (float)raw_gz / s_gyroSens;

    /* Radian/s rates for direct EKF attitude propagation */
    data->gyro_rad_x = data->gyro_x * DEG_TO_RAD;
    data->gyro_rad_y = data->gyro_y * DEG_TO_RAD;
    data->gyro_rad_z = data->gyro_z * DEG_TO_RAD;

    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadAccel(float *ax, float *ay, float *az)
{
    if (ax == NULL || ay == NULL || az == NULL) {
        return ICM42688_ERROR_INVALID_PARAM;
    }

    uint8_t raw[6];
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_ACCEL_DATA_X1, raw, 6U);

    int16_t raw_ax = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    int16_t raw_ay = (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);
    int16_t raw_az = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

    *ax = (float)raw_ax / s_accelSens;
    *ay = (float)raw_ay / s_accelSens;
    *az = (float)raw_az / s_accelSens;

    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadGyro(float *gx, float *gy, float *gz)
{
    if (gx == NULL || gy == NULL || gz == NULL) {
        return ICM42688_ERROR_INVALID_PARAM;
    }

    uint8_t raw[6];
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_GYRO_DATA_X1, raw, 6U);

    int16_t raw_gx = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    int16_t raw_gy = (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);
    int16_t raw_gz = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

    *gx = (float)raw_gx / s_gyroSens;
    *gy = (float)raw_gy / s_gyroSens;
    *gz = (float)raw_gz / s_gyroSens;

    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadTemp(float *temp_c)
{
    if (temp_c == NULL) {
        return ICM42688_ERROR_INVALID_PARAM;
    }

    uint8_t raw[2];
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_TEMP_DATA1, raw, 2U);

    int16_t raw_temp = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    *temp_c = ((float)raw_temp / ICM42688_TEMP_SENSITIVITY) + ICM42688_TEMP_OFFSET_C;

    return ICM42688_OK;
}

ICM42688_Status_t ICM42688_SetFilters(ICM42688_FilterBW_t gyro_bw,
                                      ICM42688_FilterBW_t accel_bw)
{
    uint8_t filt_cfg = (uint8_t)((((uint8_t)accel_bw & 0x0FU) << 4U) |
                                 ((uint8_t)gyro_bw & 0x0FU));
    ICM42688_WriteRegBank(ICM42688_BANK_0, ICM42688_B0_GYRO_ACCEL_CONFIG0, filt_cfg);
    return ICM42688_OK;
}

bool ICM42688_IsDataReady(void)
{
    uint8_t status = 0U;
    ICM42688_ReadRegsBank(ICM42688_BANK_0, ICM42688_B0_INT_STATUS, &status, 1U);
    return (status & ICM42688_INT_STATUS_DATA_RDY) != 0U;
}
