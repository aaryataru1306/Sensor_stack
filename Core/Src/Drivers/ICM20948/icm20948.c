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
 *   (ICM20948_CS_PIN, PA3 by default) to SPI1_ReadRegisters() /
 *   SPI1_WriteRegister().
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

/**
 * @brief Currently selected register bank on the (single) ICM20948.
 *
 * @details
 *   The ICM20948's register space is split into four banks, selected by
 *   writing to REG_BANK_SEL (0x7F). The bank selector is not preserved
 *   across a device reset, and writing it on every single register
 *   access would double the SPI traffic — so we cache the last value we
 *   wrote and skip the bank-select write when it hasn't changed.
 *
 *   0xFF is an impossible bank value chosen as the "unknown" sentinel so
 *   the first access after a reset always triggers a bank-select write.
 */
static uint8_t s_currentBank = 0xFFU;

/**
 * @brief Scale factors chosen at Init() time, used by every read to
 *        convert raw counts into physical units.
 *
 * @details
 *   Defaults match the power-on reset state of the part (±2 g, ±250 dps)
 *   so a caller that reads data before (or without) a successful
 *   ICM20948_Init() still gets a plausibly-scaled answer rather than
 *   garbage.
 */
static float s_accelSens = ICM20948_ACCEL_SENS_2G;   /* LSB/g   */
static float s_gyroSens  = ICM20948_GYRO_SENS_250DPS;/* LSB/dps */

/*============================================================================*
 *                          INTERNAL BUS PRIMITIVES                           *
 *============================================================================*/

/**
 * @brief Write one register in the currently-selected bank.
 *
 * @details
 *   Bit 7 of the address byte is the read/write selector on the
 *   ICM20948 — 1 for read, 0 for write — so it is forced to 0 here. No
 *   bank selection is performed; callers go through the Bank-aware
 *   wrappers below.
 *
 * @param[in] reg    Register address within the current bank.
 * @param[in] value  Byte to write.
 */
static void ICM20948_WriteReg(uint8_t reg, uint8_t value)
{
    SPI1_WriteRegister(ICM20948_CS_PIN, (uint8_t)(reg & 0x7FU), value);
}

/**
 * @brief Read `len` bytes starting at `reg` in the currently-selected
 *        bank.
 *
 * @details
 *   Bit 7 of the address byte is forced to 1 for a read. As with the
 *   write path, no bank selection happens here.
 *
 * @param[in]  reg  Register address within the current bank.
 * @param[out] buf  Destination for `len` bytes.
 * @param[in]  len  Number of bytes to read.
 */
static void ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    SPI1_ReadRegisters(ICM20948_CS_PIN, (uint8_t)(reg | 0x80U), buf, len);
}

/**
 * @brief Select the register bank (0..3), caching the last value so
 *        consecutive accesses in the same bank skip the write.
 *
 * @param[in] bank  Bank number 0..3.
 */
static void ICM20948_SelectBank(uint8_t bank)
{
    if (bank == s_currentBank) {
        return;
    }
    ICM20948_WriteReg(ICM20948_REG_BANK_SEL, (uint8_t)((bank << 4U) & 0x30U));
    s_currentBank = bank;
}

/**
 * @brief Select bank then write one register.
 */
static void ICM20948_WriteRegBank(uint8_t bank, uint8_t reg, uint8_t value)
{
    ICM20948_SelectBank(bank);
    ICM20948_WriteReg(reg, value);
}

/**
 * @brief Select bank then read a block of registers.
 */
static void ICM20948_ReadRegsBank(uint8_t bank, uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM20948_SelectBank(bank);
    ICM20948_ReadRegs(reg, buf, len);
}

/*============================================================================*
 *                          INTERNAL DELAY                                    *
 *============================================================================*/

/**
 * @brief Short busy-wait used during power-up and reset.
 *
 * @details
 *   The ICM20948 needs a few milliseconds after power-on and after a
 *   soft reset before it answers on the bus again. This project has a
 *   proper timer-based delay coming (TIM_interface.h / TIM_program.c,
 *   TIM6), and once it exists this loop should be replaced by a call to
 *   it. Until then, an uncalibrated spin is what's available — the
 *   driver's README documents this as a known simplification.
 *
 *   Named `icm20948_delay_ms` (lowercase, file-static) so it cannot
 *   collide with any future shared `TIM_vDelayMs` or `HAL_Delay` symbol
 *   in the same link unit.
 *
 * @param[in] ms  Rough milliseconds to spin. Not calibrated to real time.
 */
static void icm20948_delay_ms(uint32_t ms)
{
    /* ~8000 iterations per ms at 100 MHz — see header comment for why
     * this is approximate and why that's acceptable here. */
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
    ICM20948_ReadRegsBank(0U, ICM20948_B0_WHO_AM_I, &who, 1U);
    return who;
}

ICM20948_Status_t ICM20948_Init(ICM20948_AccelFS_t accel_fs,
                                ICM20948_GyroFS_t  gyro_fs)
{
    /* The SPI1 peripheral is shared with the BMP388, so it is brought
     * up exactly once, from main.c, before any sensor driver runs. This
     * driver used to call SPI1_Init() itself — that was safe when there
     * was only one SPI sensor, and is not safe now: the second sensor's
     * init would reprogram CR1 while the first was mid-transaction on
     * some future code path. Callers must call SPI1_Init() first. */

    /* 1. Software reset. */
    ICM20948_WriteRegBank(0U, ICM20948_B0_PWR_MGMT_1, ICM20948_PWR_DEVICE_RESET);
    icm20948_delay_ms(50U);

    /* Bank state is unknown after a reset — force a re-select on the
     * next access by invalidating the cache. */
    s_currentBank = 0xFFU;

    /* 2. Identity check. */
    uint8_t who = ICM20948_WhoAmI();
    if (who != ICM20948_WHO_AM_I_VALUE) {
        return ICM20948_ERROR_WHOAMI;
    }

    /* 3. Wake up: clear SLEEP, choose the gyro-referenced PLL as clock
     *    source (far more stable than the internal oscillator). */
    ICM20948_WriteRegBank(0U, ICM20948_B0_PWR_MGMT_1, ICM20948_CLKSEL_AUTO);
    icm20948_delay_ms(10U);

    /* 4. Clear any per-axis standby bits — enable all 6 axes. */
    ICM20948_WriteRegBank(0U, ICM20948_B0_PWR_MGMT_2, 0x00U);

    /* 5. Gyro full-scale range. FS_SEL occupies bits [2:1] of
     *    GYRO_CONFIG_1; bit 0 (GYRO_FCHOICE) stays 0 to bypass the
     *    on-chip DLPF, matching the original driver's behaviour. */
    uint8_t gyro_cfg = (uint8_t)(((uint8_t)gyro_fs & 0x03U) << 1U);
    ICM20948_WriteRegBank(2U, ICM20948_B2_GYRO_CONFIG_1, gyro_cfg);

    /* 6. Accel full-scale range, same bit layout in ACCEL_CONFIG. */
    uint8_t accel_cfg = (uint8_t)(((uint8_t)accel_fs & 0x03U) << 1U);
    ICM20948_WriteRegBank(2U, ICM20948_B2_ACCEL_CONFIG, accel_cfg);

    /* 7. Sample-rate dividers — leave at defaults (max ODR). */
    ICM20948_WriteRegBank(2U, ICM20948_B2_GYRO_SMPLRT_DIV,    0x00U);
    ICM20948_WriteRegBank(2U, ICM20948_B2_ACCEL_SMPLRT_DIV_1, 0x00U);
    ICM20948_WriteRegBank(2U, ICM20948_B2_ACCEL_SMPLRT_DIV_2, 0x00U);

    /* 8. Cache the scale factors for the read path. */
    switch (accel_fs) {
        case ICM20948_ACCEL_FS_2G:  s_accelSens = ICM20948_ACCEL_SENS_2G;  break;
        case ICM20948_ACCEL_FS_4G:  s_accelSens = ICM20948_ACCEL_SENS_4G;  break;
        case ICM20948_ACCEL_FS_8G:  s_accelSens = ICM20948_ACCEL_SENS_8G;  break;
        case ICM20948_ACCEL_FS_16G: s_accelSens = ICM20948_ACCEL_SENS_16G; break;
        default:                    s_accelSens = ICM20948_ACCEL_SENS_2G;  break;
    }

    switch (gyro_fs) {
        case ICM20948_GYRO_FS_250DPS:  s_gyroSens = ICM20948_GYRO_SENS_250DPS;  break;
        case ICM20948_GYRO_FS_500DPS:  s_gyroSens = ICM20948_GYRO_SENS_500DPS;  break;
        case ICM20948_GYRO_FS_1000DPS: s_gyroSens = ICM20948_GYRO_SENS_1000DPS; break;
        case ICM20948_GYRO_FS_2000DPS: s_gyroSens = ICM20948_GYRO_SENS_2000DPS; break;
        default:                       s_gyroSens = ICM20948_GYRO_SENS_250DPS;  break;
    }

    /* 9. Return to Bank 0 as the resting bank. */
    ICM20948_SelectBank(0U);

    return ICM20948_OK;
}

void ICM20948_ReadAccel(float *ax, float *ay, float *az)
{
    if (ax == 0 || ay == 0 || az == 0) {
        return;
    }

    uint8_t raw[6];
    ICM20948_ReadRegsBank(0U, ICM20948_B0_ACCEL_XOUT_H, raw, 6U);

    int16_t rawX = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    int16_t rawY = (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);
    int16_t rawZ = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

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
    ICM20948_ReadRegsBank(0U, ICM20948_B0_GYRO_XOUT_H, raw, 6U);

    int16_t rawX = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    int16_t rawY = (int16_t)(((uint16_t)raw[2] << 8U) | (uint16_t)raw[3]);
    int16_t rawZ = (int16_t)(((uint16_t)raw[4] << 8U) | (uint16_t)raw[5]);

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
    ICM20948_ReadRegsBank(0U, ICM20948_B0_TEMP_OUT_H, raw, 2U);

    int16_t rawT = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);

    /* From the datasheet §4.9:
     *   Temp [°C] = (raw / 333.87) + 21.0
     * The two constants are named in icm20948_private.h so the formula
     * reads as written rather than as a pair of magic numbers. */
    *temp_c = ((float)rawT / ICM20948_TEMP_SENSITIVITY) + ICM20948_TEMP_OFFSET_C;
}

void ICM20948_ReadAll(ICM20948_Data_t *data)
{
    if (data == 0) {
        return;
    }
    ICM20948_ReadAccel(&data->accel_x, &data->accel_y, &data->accel_z);
    ICM20948_ReadGyro (&data->gyro_x,  &data->gyro_y,  &data->gyro_z);
    ICM20948_ReadTemp (&data->temp_c);
}
