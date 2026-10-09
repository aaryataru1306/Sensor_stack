/*
 * icm20948.c
 *
 * Driver for TDK InvenSense ICM-20948 (9-axis IMU) over SPI
 * Built on top of the user's own register-level SPI1 driver (spi_driver.h/.c)
 * Target: STM32F411CEU6
 *
 * Place this file next to spi_driver.c (e.g. Core/Src)
 */

#include "icm20948.h"

/* ================= SIMPLE BLOCKING DELAY =================
 * The user's SPI driver is pure CMSIS/register-level (no HAL), so there is
 * no HAL_Delay() guaranteed to exist. This is a rough busy-wait loop.
 *
 * If you DO have HAL initialized elsewhere in your project (HAL_Delay
 * available via SysTick), delete this function and #include "stm32f4xx_hal.h"
 * instead, then replace ICM20948_DelayMs(x) calls below with HAL_Delay(x).
 *
 * As written, this loop count is a rough approximation for a 100MHz core
 * clock and is NOT cycle-accurate. It is only used for power-up/reset
 * delays, so exact timing is not critical (over-waiting is harmless).
 */
static void ICM20948_DelayMs(uint32_t ms)
{
    /* ~100000 loop iterations per ms at 100MHz, adjust if your SYSCLK differs */
    volatile uint32_t count = ms * 8000U;
    while (count--)
    {
        __asm__("nop");
    }
}

/* Keep track of currently selected register bank to avoid unnecessary SPI writes */
static uint8_t s_currentBank = 0xFF; /* invalid value forces first select */

/* Sensitivity scale factors, set at Init() according to chosen full-scale range */
static float s_accelSens = 16384.0f; /* LSB/g   for +-2g   */
static float s_gyroSens  = 131.0f;   /* LSB/dps for 250dps */

/* ================= LOW LEVEL WRAPPERS AROUND USER'S SPI DRIVER =================
 * ICM20948 SPI protocol: bit7 of the first byte = 1 for READ, 0 for WRITE.
 * The user's spi_driver.c sends reg_addr exactly as given (no bit shifting),
 * so we set/clear that bit HERE, keeping spi_driver.c generic and untouched.
 */

/* Write a single register in the CURRENT bank (no bank select here) */
#define ICM20948_CS_PIN   1   /* غيّر الرقم ده حسب تعريف الـ CS في spi_driver.h */

static void ICM20948_WriteReg(uint8_t reg, uint8_t value)
{
    SPI1_WriteRegister(SPI1_CS_ICM20948_PIN, reg & 0x7F, value);
}

static void ICM20948_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    SPI1_ReadRegisters(SPI1_CS_ICM20948_PIN, reg | 0x80, buf, len);
}
/* Select the register bank (0, 1, 2 or 3). Cached so we don't spam SPI writes. */
static void ICM20948_SelectBank(uint8_t bank)
{
    if (bank == s_currentBank)
    {
        return;
    }

    ICM20948_WriteReg(ICM20948_REG_BANK_SEL, (uint8_t)((bank << 4) & 0x30));
    s_currentBank = bank;
}

/* Convenience wrapper: select bank then write register */
static void ICM20948_WriteRegBank(uint8_t bank, uint8_t reg, uint8_t value)
{
    ICM20948_SelectBank(bank);
    ICM20948_WriteReg(reg, value);
}

/* Convenience wrapper: select bank then read registers */
static void ICM20948_ReadRegsBank(uint8_t bank, uint8_t reg, uint8_t *buf, uint16_t len)
{
    ICM20948_SelectBank(bank);
    ICM20948_ReadRegs(reg, buf, len);
}

/* ================= PUBLIC API ================= */

uint8_t ICM20948_WhoAmI(void)
{
    uint8_t whoAmI = 0;
    ICM20948_ReadRegsBank(0, ICM20948_B0_WHO_AM_I, &whoAmI, 1);
    return whoAmI;
}

ICM20948_Status_t ICM20948_Init(ICM20948_AccelFS_t accel_fs, ICM20948_GyroFS_t gyro_fs)
{
    /* 0) Bring up the SPI1 peripheral + GPIOs (this configures PA4-PA7) */
    SPI1_Init();
    SPI_CS_Init(SPI1_CS_ICM20948_PIN);
    SPI_CS_Deselect(SPI1_CS_ICM20948_PIN);
    ICM20948_DelayMs(10);

    /* 1) Software reset (Bank 0, PWR_MGMT_1 bit 7) */
    ICM20948_WriteRegBank(0, ICM20948_B0_PWR_MGMT_1, 0x80);
    ICM20948_DelayMs(50); /* wait for reset to complete */
    s_currentBank = 0xFF; /* bank state is unknown after reset, force reselect */

    /* 2) Verify WHO_AM_I */
    uint8_t whoAmI = ICM20948_WhoAmI();
    if (whoAmI != ICM20948_WHO_AM_I_VALUE)
    {
        return ICM20948_ERROR_WHOAMI;
    }

    /* 3) Wake up device, auto-select best clock source (PLL if available) */
    ICM20948_WriteRegBank(0, ICM20948_B0_PWR_MGMT_1, 0x01);
    ICM20948_DelayMs(10);

    /* 4) Enable accel + gyro (disable low power / standby bits) */
    ICM20948_WriteRegBank(0, ICM20948_B0_PWR_MGMT_2, 0x00);

    /* 5) Configure Gyro full scale range (Bank 2, GYRO_CONFIG_1) */
    uint8_t gyroCfg = (uint8_t)((gyro_fs & 0x03) << 1); /* bits [2:1] = FS_SEL, DLPF disabled */
    ICM20948_WriteRegBank(2, ICM20948_B2_GYRO_CONFIG_1, gyroCfg);

    /* 6) Configure Accel full scale range (Bank 2, ACCEL_CONFIG) */
    uint8_t accelCfg = (uint8_t)((accel_fs & 0x03) << 1); /* bits [2:1] = FS_SEL, DLPF disabled */
    ICM20948_WriteRegBank(2, ICM20948_B2_ACCEL_CONFIG, accelCfg);

    /* 7) Set sample rate dividers (optional, defaults are fine to start) */
    ICM20948_WriteRegBank(2, ICM20948_B2_GYRO_SMPLRT_DIV, 0x00);
    ICM20948_WriteRegBank(2, ICM20948_B2_ACCEL_SMPLRT_DIV_1, 0x00);
    ICM20948_WriteRegBank(2, ICM20948_B2_ACCEL_SMPLRT_DIV_2, 0x00);

    /* 8) Store sensitivity scale factors for conversion later */
    switch (accel_fs)
    {
        case ICM20948_ACCEL_FS_2G:  s_accelSens = 16384.0f; break;
        case ICM20948_ACCEL_FS_4G:  s_accelSens = 8192.0f;  break;
        case ICM20948_ACCEL_FS_8G:  s_accelSens = 4096.0f;  break;
        case ICM20948_ACCEL_FS_16G: s_accelSens = 2048.0f;  break;
    }

    switch (gyro_fs)
    {
        case ICM20948_GYRO_FS_250DPS:  s_gyroSens = 131.0f;  break;
        case ICM20948_GYRO_FS_500DPS:  s_gyroSens = 65.5f;   break;
        case ICM20948_GYRO_FS_1000DPS: s_gyroSens = 32.8f;   break;
        case ICM20948_GYRO_FS_2000DPS: s_gyroSens = 16.4f;   break;
    }

    /* Return to Bank 0 as the "resting" bank */
    ICM20948_SelectBank(0);

    return ICM20948_OK;
}

void ICM20948_ReadAccel(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    ICM20948_ReadRegsBank(0, ICM20948_B0_ACCEL_XOUT_H, raw, 6);

    int16_t rawX = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t rawY = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t rawZ = (int16_t)((raw[4] << 8) | raw[5]);

    *ax = (float)rawX / s_accelSens;
    *ay = (float)rawY / s_accelSens;
    *az = (float)rawZ / s_accelSens;
}

void ICM20948_ReadGyro(float *gx, float *gy, float *gz)
{
    uint8_t raw[6];
    ICM20948_ReadRegsBank(0, ICM20948_B0_GYRO_XOUT_H, raw, 6);

    int16_t rawX = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t rawY = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t rawZ = (int16_t)((raw[4] << 8) | raw[5]);

    *gx = (float)rawX / s_gyroSens;
    *gy = (float)rawY / s_gyroSens;
    *gz = (float)rawZ / s_gyroSens;
}

void ICM20948_ReadTemp(float *temp_c)
{
    uint8_t raw[2];
    ICM20948_ReadRegsBank(0, ICM20948_B0_TEMP_OUT_H, raw, 2);

    int16_t rawT = (int16_t)((raw[0] << 8) | raw[1]);

    /* From datasheet: Temp_degC = (RAW / 333.87) + 21 */
    *temp_c = ((float)rawT / 333.87f) + 21.0f;
}

void ICM20948_ReadAll(ICM20948_Data_t *data)
{
    ICM20948_ReadAccel(&data->accel_x, &data->accel_y, &data->accel_z);
    ICM20948_ReadGyro(&data->gyro_x, &data->gyro_y, &data->gyro_z);
    ICM20948_ReadTemp(&data->temp_c);
}
