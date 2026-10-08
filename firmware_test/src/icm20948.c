/**
 ******************************************************************************
 * @file    icm20948.c
 * @brief   Implementation of the TDK InvenSense ICM-20948 driver declared
 *          in icm20948.h.
 * @details
 *   Pipeline: ICM20948_ReadAll() -> ICM20948_ReadAccel/Gyro/Temp()
 *             (SPI bursts) -> convert with the scale factors chosen at
 *             Init() -> caller-supplied ICM20948_Data_t.
 *
 *   All bus traffic goes through the shared bare-metal SPI1 driver in
 *   Protocols/SPI/ (spi_driver.h). The ICM20948 shares SPI1 with the
 *   BMP388, so every transaction passes this driver's chip-select pin
 *   (ICM20948_CS_PIN, PA1) to SPI1_ReadRegisters() / SPI1_WriteRegister().
 *
 *   The SPI peripheral itself is brought up once, in main.c, via
 *   SPI1_Init(). This driver does NOT do it — see ICM20948_Init()'s
 *   comment for why.
 *
 *   Bank state is cached in a file-static variable. That is a deliberate
 *   exception to the "no static mutable state" rule the rest of this
 *   project follows: the bank selector on the ICM20948 is a property of
 *   the *chip*, not of a driver instance, and there is exactly one
 *   ICM20948 on the bus. A second instance would need its own bank
 *   state, and at that point this would become a per-handle field.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#include "icm20948.h"
#include "icm20948_private.h"
#include "icm20948_config.h"
#include "spi_driver.h"

/*============================================================================*
 *                          INTERNAL BUS WRAPPERS                             *
 *============================================================================*/

static uint8_t s_currentBank = 0xFFU;

static float s_accelSens = ICM20948_ACCEL_SENS_2G;
static float s_gyroSens  = ICM20948_GYRO_SENS_250DPS;

/*============================================================================*
 *                          INTERNAL BUS PRIMITIVES                           *
 *============================================================================*/

static void ICM20948_WriteReg(uint8_t reg, uint8_t value)
{
    SPI1_WriteRegister(ICM20948_CS_PIN, (uint8_t)(reg & 0x7FU), value);
}

static void ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    SPI1_ReadRegisters(ICM20948_CS_PIN, (uint8_t)(reg | 0x80U), buf, len);
}

static void ICM20948_SelectBank(uint8_t bank)
{
    if (bank == s_currentBank) {
        return;
    }

    ICM20948_WriteReg(
        ICM20948_REG_BANK_SEL,
        (uint8_t)((bank << 4U) & 0x30U)
    );

    s_currentBank = bank;
}

static void ICM20948_WriteRegBank(uint8_t bank, uint8_t reg, uint8_t value)
{
    ICM20948_SelectBank(bank);
    ICM20948_WriteReg(reg, value);
}

static void ICM20948_ReadRegsBank(
    uint8_t bank,
    uint8_t reg,
    uint8_t *buf,
    uint16_t len)
{
    ICM20948_SelectBank(bank);
    ICM20948_ReadRegs(reg, buf, len);
}

/*============================================================================*
 *                          INTERNAL DELAY                                    *
 *============================================================================*/

static void icm20948_delay_ms(uint32_t ms)
{
    volatile uint32_t count = ms * 8000U;

    while (count--) {
        __asm__("nop");
    }
}

/*============================================================================*
 *                          PUBLIC API IMPLEMENTATION                         *
 *============================================================================*/

uint8_t ICM20948_WhoAmI(void)
{
    uint8_t who = 0U;

    ICM20948_ReadRegsBank(
        0U,
        ICM20948_B0_WHO_AM_I,
        &who,
        1U
    );

    return who;
}

ICM20948_Status_t ICM20948_Init(
    ICM20948_AccelFS_t accel_fs,
    ICM20948_GyroFS_t gyro_fs)
{
    /* Validate full-scale range selections before programming registers. */
    if (accel_fs > ICM20948_ACCEL_FS_16G ||
        gyro_fs > ICM20948_GYRO_FS_2000DPS) {
        return ICM20948_ERROR_INVALID_PARAM;
    }

    /* 1. Software reset. */
    ICM20948_WriteRegBank(
        0U,
        ICM20948_B0_PWR_MGMT_1,
        ICM20948_PWR_DEVICE_RESET
    );

    icm20948_delay_ms(50U);

    /* Bank state is unknown after reset. */
    s_currentBank = 0xFFU;

    /* 2. Identity check. */
    uint8_t who = ICM20948_WhoAmI();

    if (who != ICM20948_WHO_AM_I_VALUE) {
        return ICM20948_ERROR_WHOAMI;
    }

    /* 3. Wake up and select clock source. */
    ICM20948_WriteRegBank(
        0U,
        ICM20948_B0_PWR_MGMT_1,
        ICM20948_CLKSEL_AUTO
    );

    icm20948_delay_ms(10U);

    /* 4. Enable all axes. */
    ICM20948_WriteRegBank(
        0U,
        ICM20948_B0_PWR_MGMT_2,
        0x00U
    );

    /* 5. Configure gyro full-scale range. */
    uint8_t gyro_cfg =
        (uint8_t)(((uint8_t)gyro_fs & 0x03U) << 1U);

    ICM20948_WriteRegBank(
        2U,
        ICM20948_B2_GYRO_CONFIG_1,
        gyro_cfg
    );

    /* 6. Configure accelerometer full-scale range. */
    uint8_t accel_cfg =
        (uint8_t)(((uint8_t)accel_fs & 0x03U) << 1U);

    ICM20948_WriteRegBank(
        2U,
        ICM20948_B2_ACCEL_CONFIG,
        accel_cfg
    );

    /* 7. Sample-rate dividers. */
    ICM20948_WriteRegBank(
        2U,
        ICM20948_B2_GYRO_SMPLRT_DIV,
        0x00U
    );

    ICM20948_WriteRegBank(
        2U,
        ICM20948_B2_ACCEL_SMPLRT_DIV_1,
        0x00U
    );

    ICM20948_WriteRegBank(
        2U,
        ICM20948_B2_ACCEL_SMPLRT_DIV_2,
        0x00U
    );

    /* 8. Cache scale factors. */
    switch (accel_fs) {
        case ICM20948_ACCEL_FS_2G:
            s_accelSens = ICM20948_ACCEL_SENS_2G;
            break;

        case ICM20948_ACCEL_FS_4G:
            s_accelSens = ICM20948_ACCEL_SENS_4G;
            break;

        case ICM20948_ACCEL_FS_8G:
            s_accelSens = ICM20948_ACCEL_SENS_8G;
            break;

        case ICM20948_ACCEL_FS_16G:
            s_accelSens = ICM20948_ACCEL_SENS_16G;
            break;

        default:
            s_accelSens = ICM20948_ACCEL_SENS_2G;
            break;
    }

    switch (gyro_fs) {
        case ICM20948_GYRO_FS_250DPS:
            s_gyroSens = ICM20948_GYRO_SENS_250DPS;
            break;

        case ICM20948_GYRO_FS_500DPS:
            s_gyroSens = ICM20948_GYRO_SENS_500DPS;
            break;

        case ICM20948_GYRO_FS_1000DPS:
            s_gyroSens = ICM20948_GYRO_SENS_1000DPS;
            break;

        case ICM20948_GYRO_FS_2000DPS:
            s_gyroSens = ICM20948_GYRO_SENS_2000DPS;
            break;

        default:
            s_gyroSens = ICM20948_GYRO_SENS_250DPS;
            break;
    }

    /* 9. Return to Bank 0. */
    ICM20948_SelectBank(0U);

    return ICM20948_OK;
}

void ICM20948_ReadAccel(float *ax, float *ay, float *az)
{
    if (ax == 0 || ay == 0 || az == 0) {
        return;
    }

    uint8_t raw[6];

    ICM20948_ReadRegsBank(
        0U,
        ICM20948_B0_ACCEL_XOUT_H,
        raw,
        6U
    );

    int16_t rawX =
        (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);

    int16_t rawY =
        (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);

    int16_t rawZ =
        (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

    *ax = (float)rawX / s_accelSens;
    *ay = (float)rawY / s_accelSens;
    *az = (float)rawZ / s_accelSens;
}

void ICM20948_ReadGyro(float *gx, float *gy, float *gz)
{
    if (gx == 0 || gy == 0 || gz == 0) {
        return;
    }

    uint8_t raw[6];

    ICM20948_ReadRegsBank(
        0U,
        ICM20948_B0_GYRO_XOUT_H,
        raw,
        6U
    );

    int16_t rawX =
        (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);

    int16_t rawY =
        (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);

    int16_t rawZ =
        (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

    *gx = (float)rawX / s_gyroSens;
    *gy = (float)rawY / s_gyroSens;
    *gz = (float)rawZ / s_gyroSens;
}

void ICM20948_ReadTemp(float *temp_c)
{
    if (temp_c == 0) {
        return;
    }

    uint8_t raw[2];

    ICM20948_ReadRegsBank(
        0U,
        ICM20948_B0_TEMP_OUT_H,
        raw,
        2U
    );

    int16_t rawT =
        (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);

    /* Temperature formula:
     * Temp [°C] = (raw / 333.87) + 21.0
     */
    *temp_c =
        ((float)rawT / ICM20948_TEMP_SENSITIVITY)
        + ICM20948_TEMP_OFFSET_C;
}

void ICM20948_ReadAll(ICM20948_Data_t *data)
{
    if (data == 0) {
        return;
    }

    uint8_t raw[14];

    ICM20948_ReadRegsBank(
        0U,
        ICM20948_B0_ACCEL_XOUT_H,
        raw,
        14U
    );

    int16_t raw_ax =
        (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);

    int16_t raw_ay =
        (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);

    int16_t raw_az =
        (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);

    int16_t raw_gx =
        (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);

    int16_t raw_gy =
        (int16_t)(((uint16_t)raw[8] << 8U) | raw[9]);

    int16_t raw_gz =
        (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);

    int16_t raw_temp =
        (int16_t)(((uint16_t)raw[12] << 8U) | raw[13]);

    data->accel_x = (float)raw_ax / s_accelSens;
    data->accel_y = (float)raw_ay / s_accelSens;
    data->accel_z = (float)raw_az / s_accelSens;

    data->gyro_x = (float)raw_gx / s_gyroSens;
    data->gyro_y = (float)raw_gy / s_gyroSens;
    data->gyro_z = (float)raw_gz / s_gyroSens;

    data->temp_c =
        ((float)raw_temp / ICM20948_TEMP_SENSITIVITY)
        + ICM20948_TEMP_OFFSET_C;
}
