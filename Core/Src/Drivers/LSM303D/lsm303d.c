/**
 ******************************************************************************
 * @file    lsm303d.c
 * @brief   Implementation of the STMicroelectronics LSM303D 6-axis eCompass driver
 *          (3D Accelerometer + 3D Magnetometer + Temperature).
 * @details
 *   Pipeline:
 *     - Accel: 6-byte burst read from OUT_X_L_A (0x28) -> int16 conversion ->
 *              sensitivity scaling -> zero-offset correction -> g & m/s².
 *     - Mag:   6-byte burst read from OUT_X_L_M (0x08) -> int16 conversion ->
 *              sensitivity scaling -> Gauss & µT.
 *     - Temp:  2-byte read from TEMP_OUT_L (0x05) -> int16 conversion ->
 *              0.125 °C/LSB + baseline offset -> °C.
 *
 *   Multi-Protocol Bus Support:
 *     - SPI: Driven via shared bare-metal SPI1 driver (spi_driver.h).
 *            Command byte: Bit 7 = 1 (Read), Bit 6 = 1 (Auto-increment),
 *            Bits[5:0] = Register address.
 *     - I2C: Driven via shared bare-metal I2C driver (i2c_interface.h).
 *            Sub-address: Bit 7 = 1 (Auto-increment) for multi-byte bursts.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#include "lsm303d.h"
#include "lsm303d_private.h"
#include "spi_driver.h"
#include "i2c_interface.h"

/*============================================================================*
 *                          INTERNAL DRIVER STATE                             *
 *============================================================================*/

static LSM303D_BusConfig_t s_bus_cfg;
static float               s_accel_sens   = LSM303D_ACCEL_SENS_2G;
static float               s_mag_sens     = LSM303D_MAG_SENS_4GAUSS;
static float               s_accel_offset[3] = { 0.0f, 0.0f, 0.0f };
static bool                s_initialized  = false;

/*============================================================================*
 *                          INTERNAL BUS PRIMITIVES                           *
 *============================================================================*/

static void lsm303d_delay_ms(uint32_t ms)
{
    /* Spin loop calibrated for STM32F411 at 100 MHz SYSCLK */
    for (volatile uint32_t i = 0U; i < ms; i++) {
        for (volatile uint32_t j = 0U; j < 20000U; j++) {
            __asm__("nop");
        }
    }
}

/**
 * @brief Write a single byte to an LSM303D register over the selected bus.
 */
static LSM303D_Status_t LSM303D_WriteReg(uint8_t reg, uint8_t value)
{
    if (s_bus_cfg.bus_type == LSM303D_BUS_SPI) {
        /* SPI Write: Bit 7 = 0, Bit 6 = 0, Reg in bits 5..0 */
        uint8_t addr = (uint8_t)(reg & LSM303D_SPI_ADDR_MASK);
        SPI1_WriteRegister(s_bus_cfg.spi_cs_pin, addr, value);
        return LSM303D_OK;
    } else if (s_bus_cfg.bus_type == LSM303D_BUS_I2C) {
        if (s_bus_cfg.i2c_cfg == 0) {
            return LSM303D_ERROR_INVALID_PARAM;
        }
        uint8_t buf[2];
        buf[0] = reg;
        buf[1] = value;
        ErrorState_t err = I2C_enumMasterTransmit(s_bus_cfg.i2c_cfg,
                                                  s_bus_cfg.i2c_addr,
                                                  buf,
                                                  2U);
        return (err == OK) ? LSM303D_OK : LSM303D_ERROR_BUS;
    }

    return LSM303D_ERROR_INVALID_PARAM;
}

/**
 * @brief Read a single byte from an LSM303D register over the selected bus.
 */
static LSM303D_Status_t LSM303D_ReadReg(uint8_t reg, uint8_t *val)
{
    if (val == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    if (s_bus_cfg.bus_type == LSM303D_BUS_SPI) {
        /* SPI Read: Bit 7 = 1, Bit 6 = 0, Reg in bits 5..0 */
        uint8_t addr = (uint8_t)(LSM303D_SPI_READ_BIT | (reg & LSM303D_SPI_ADDR_MASK));
        SPI1_ReadRegisters(s_bus_cfg.spi_cs_pin, addr, val, 1U);
        return LSM303D_OK;
    } else if (s_bus_cfg.bus_type == LSM303D_BUS_I2C) {
        if (s_bus_cfg.i2c_cfg == 0) {
            return LSM303D_ERROR_INVALID_PARAM;
        }
        uint8_t tx = reg;
        ErrorState_t err = I2C_enumMasterTransmitReceive(s_bus_cfg.i2c_cfg,
                                                         s_bus_cfg.i2c_addr,
                                                         &tx,
                                                         1U,
                                                         val,
                                                         1U);
        return (err == OK) ? LSM303D_OK : LSM303D_ERROR_BUS;
    }

    return LSM303D_ERROR_INVALID_PARAM;
}

/**
 * @brief Burst-read contiguous bytes from LSM303D registers with auto-increment.
 */
static LSM303D_Status_t LSM303D_ReadBurst(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    if (buf == 0 || len == 0U) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    if (s_bus_cfg.bus_type == LSM303D_BUS_SPI) {
        /* SPI Multi-read: Bit 7 = 1 (Read), Bit 6 = 1 (Auto-increment) */
        uint8_t addr = (uint8_t)(LSM303D_SPI_READ_BIT | LSM303D_SPI_MS_BIT | (start_reg & LSM303D_SPI_ADDR_MASK));
        SPI1_ReadRegisters(s_bus_cfg.spi_cs_pin, addr, buf, len);
        return LSM303D_OK;
    } else if (s_bus_cfg.bus_type == LSM303D_BUS_I2C) {
        if (s_bus_cfg.i2c_cfg == 0) {
            return LSM303D_ERROR_INVALID_PARAM;
        }
        /* I2C Multi-read: Bit 7 = 1 (Auto-increment sub-address) */
        uint8_t tx = (uint8_t)(start_reg | LSM303D_I2C_AUTO_INC_MASK);
        ErrorState_t err = I2C_enumMasterTransmitReceive(s_bus_cfg.i2c_cfg,
                                                         s_bus_cfg.i2c_addr,
                                                         &tx,
                                                         1U,
                                                         buf,
                                                         len);
        return (err == OK) ? LSM303D_OK : LSM303D_ERROR_BUS;
    }

    return LSM303D_ERROR_INVALID_PARAM;
}

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

uint8_t LSM303D_WhoAmI(void)
{
    uint8_t id = 0U;
    (void)LSM303D_ReadReg(LSM303D_REG_WHO_AM_I, &id);
    return id;
}

LSM303D_Status_t LSM303D_SoftReset(void)
{
    /* Write BOOT bit in CTRL_REG0 to reboot memory content */
    LSM303D_Status_t status = LSM303D_WriteReg(LSM303D_REG_CTRL_REG0, LSM303D_CTRL0_BOOT);
    lsm303d_delay_ms(15U);
    return status;
}

LSM303D_Status_t LSM303D_Init(const LSM303D_BusConfig_t *bus_cfg,
                              LSM303D_AccelFS_t          accel_fs,
                              LSM303D_AccelODR_t         accel_odr,
                              LSM303D_MagFS_t            mag_fs,
                              LSM303D_MagODR_t           mag_odr)
{
    if (bus_cfg == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    /* Store bus configuration */
    s_bus_cfg = *bus_cfg;
    s_initialized = false;

    /* If I2C bus is selected, ensure default address is valid */
    if (s_bus_cfg.bus_type == LSM303D_BUS_I2C) {
        if (s_bus_cfg.i2c_addr != LSM303D_I2C_ADDR_LOW &&
            s_bus_cfg.i2c_addr != LSM303D_I2C_ADDR_HIGH) {
            s_bus_cfg.i2c_addr = LSM303D_I2C_ADDR_LOW;
        }
    }

    /* Small startup settling delay */
    lsm303d_delay_ms(10U);

    /* 1. Verify WHO_AM_I identity (0x0F == 0x49) */
    uint8_t whoami = LSM303D_WhoAmI();
    if (whoami != LSM303D_WHO_AM_I_VAL) {
        return LSM303D_ERROR_WHOAMI;
    }

    /* 2. Soft reset memory registers */
    (void)LSM303D_SoftReset();

    /* 3. Assign accelerometer sensitivity scale factor based on full-scale range */
    switch (accel_fs) {
        case LSM303D_ACCEL_FS_2G:  s_accel_sens = LSM303D_ACCEL_SENS_2G;  break;
        case LSM303D_ACCEL_FS_4G:  s_accel_sens = LSM303D_ACCEL_SENS_4G;  break;
        case LSM303D_ACCEL_FS_6G:  s_accel_sens = LSM303D_ACCEL_SENS_6G;  break;
        case LSM303D_ACCEL_FS_8G:  s_accel_sens = LSM303D_ACCEL_SENS_8G;  break;
        case LSM303D_ACCEL_FS_16G: s_accel_sens = LSM303D_ACCEL_SENS_16G; break;
        default:                   s_accel_sens = LSM303D_ACCEL_SENS_2G;  break;
    }

    /* 4. Assign magnetometer sensitivity scale factor based on full-scale range */
    switch (mag_fs) {
        case LSM303D_MAG_FS_2GAUSS:  s_mag_sens = LSM303D_MAG_SENS_2GAUSS;  break;
        case LSM303D_MAG_FS_4GAUSS:  s_mag_sens = LSM303D_MAG_SENS_4GAUSS;  break;
        case LSM303D_MAG_FS_8GAUSS:  s_mag_sens = LSM303D_MAG_SENS_8GAUSS;  break;
        case LSM303D_MAG_FS_12GAUSS: s_mag_sens = LSM303D_MAG_SENS_12GAUSS; break;
        default:                     s_mag_sens = LSM303D_MAG_SENS_4GAUSS;  break;
    }

    /* 5. Configure CTRL_REG1 (0x20):
     *    AODR[3:0] | BDU (Bit 3) | AZEN (Bit 2) | AYEN (Bit 1) | AXEN (Bit 0)
     *    BDU = 1 prevents reading tearing across MSB/LSB updates.
     */
    uint8_t ctrl1 = (uint8_t)(((uint8_t)accel_odr << LSM303D_CTRL1_AODR_POS) |
                              LSM303D_CTRL1_BDU |
                              LSM303D_CTRL1_ALL_AXES);
    if (LSM303D_WriteReg(LSM303D_REG_CTRL_REG1, ctrl1) != LSM303D_OK) {
        return LSM303D_ERROR_BUS;
    }

    /* 6. Configure CTRL_REG2 (0x21):
     *    ABW (Anti-alias BW = 50Hz for vibration suppression) | AFS | AST (Normal) | SIM (4-wire)
     */
    uint8_t ctrl2 = (uint8_t)(((uint8_t)LSM303D_ACCEL_BW_50HZ << LSM303D_CTRL2_ABW_POS) |
                              ((uint8_t)accel_fs << LSM303D_CTRL2_AFS_POS));
    if (LSM303D_WriteReg(LSM303D_REG_CTRL_REG2, ctrl2) != LSM303D_OK) {
        return LSM303D_ERROR_BUS;
    }

    /* 7. Configure CTRL_REG5 (0x24):
     *    TEMP_EN = 1 (enable temperature sensor) | M_RES = 11 (high resolution) | M_ODR
     */
    uint8_t ctrl5 = (uint8_t)(LSM303D_CTRL5_TEMP_EN |
                              LSM303D_CTRL5_M_RES_HIGH |
                              ((uint8_t)mag_odr << LSM303D_CTRL5_M_ODR_POS));
    if (LSM303D_WriteReg(LSM303D_REG_CTRL_REG5, ctrl5) != LSM303D_OK) {
        return LSM303D_ERROR_BUS;
    }

    /* 8. Configure CTRL_REG6 (0x25):
     *    MFS[1:0] (Magnetometer full-scale range)
     */
    uint8_t ctrl6 = (uint8_t)((uint8_t)mag_fs << LSM303D_CTRL6_MFS_POS);
    if (LSM303D_WriteReg(LSM303D_REG_CTRL_REG6, ctrl6) != LSM303D_OK) {
        return LSM303D_ERROR_BUS;
    }

    /* 9. Configure CTRL_REG7 (0x26):
     *    MD[1:0] = 00b (Continuous-conversion mode for magnetic sensor)
     */
    uint8_t ctrl7 = (uint8_t)LSM303D_CTRL7_MD_CONTINUOUS;
    if (LSM303D_WriteReg(LSM303D_REG_CTRL_REG7, ctrl7) != LSM303D_OK) {
        return LSM303D_ERROR_BUS;
    }

    s_initialized = true;
    return LSM303D_OK;
}

LSM303D_Status_t LSM303D_InitDefaultSPI(uint8_t cs_pin)
{
    LSM303D_BusConfig_t bus_cfg;
    bus_cfg.bus_type   = LSM303D_BUS_SPI;
    bus_cfg.spi_cs_pin = cs_pin;
    bus_cfg.i2c_cfg    = 0;
    bus_cfg.i2c_addr   = 0U;

    return LSM303D_Init(&bus_cfg,
                        LSM303D_ACCEL_FS_2G,
                        LSM303D_ACCEL_ODR_100HZ,
                        LSM303D_MAG_FS_4GAUSS,
                        LSM303D_MAG_ODR_100HZ);
}

LSM303D_Status_t LSM303D_InitDefaultI2C(I2C_Config_t *i2c_cfg, uint8_t i2c_addr)
{
    LSM303D_BusConfig_t bus_cfg;
    bus_cfg.bus_type   = LSM303D_BUS_I2C;
    bus_cfg.spi_cs_pin = 0U;
    bus_cfg.i2c_cfg    = i2c_cfg;
    bus_cfg.i2c_addr   = i2c_addr;

    return LSM303D_Init(&bus_cfg,
                        LSM303D_ACCEL_FS_2G,
                        LSM303D_ACCEL_ODR_100HZ,
                        LSM303D_MAG_FS_4GAUSS,
                        LSM303D_MAG_ODR_100HZ);
}

/*============================================================================*
 *                        ACCELEROMETER READ FUNCTIONS                        *
 *============================================================================*/

LSM303D_Status_t LSM303D_ReadAccelRaw(int16_t *rx, int16_t *ry, int16_t *rz)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }
    if (rx == 0 || ry == 0 || rz == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    uint8_t buf[6];
    LSM303D_Status_t status = LSM303D_ReadBurst(LSM303D_REG_OUT_X_L_A, buf, 6U);
    if (status != LSM303D_OK) {
        return status;
    }

    *rx = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    *ry = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    *rz = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);

    return LSM303D_OK;
}

LSM303D_Status_t LSM303D_ReadAccel(float *ax, float *ay, float *az)
{
    if (ax == 0 || ay == 0 || az == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    int16_t rx, ry, rz;
    LSM303D_Status_t status = LSM303D_ReadAccelRaw(&rx, &ry, &rz);
    if (status != LSM303D_OK) {
        return status;
    }

    *ax = ((float)rx * s_accel_sens) - s_accel_offset[0];
    *ay = ((float)ry * s_accel_sens) - s_accel_offset[1];
    *az = ((float)rz * s_accel_sens) - s_accel_offset[2];

    return LSM303D_OK;
}

LSM303D_Status_t LSM303D_ReadAccelMps2(float *ax, float *ay, float *az)
{
    float gx, gy, gz;
    LSM303D_Status_t status = LSM303D_ReadAccel(&gx, &gy, &gz);
    if (status != LSM303D_OK) {
        return status;
    }

    *ax = gx * GRAVITY_EARTH_MPS2;
    *ay = gy * GRAVITY_EARTH_MPS2;
    *az = gz * GRAVITY_EARTH_MPS2;

    return LSM303D_OK;
}

/*============================================================================*
 *                        MAGNETOMETER READ FUNCTIONS                         *
 *============================================================================*/

LSM303D_Status_t LSM303D_ReadMagRaw(int16_t *rx, int16_t *ry, int16_t *rz)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }
    if (rx == 0 || ry == 0 || rz == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    uint8_t buf[6];
    LSM303D_Status_t status = LSM303D_ReadBurst(LSM303D_REG_OUT_X_L_M, buf, 6U);
    if (status != LSM303D_OK) {
        return status;
    }

    *rx = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    *ry = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    *rz = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);

    return LSM303D_OK;
}

LSM303D_Status_t LSM303D_ReadMag(float *mx, float *my, float *mz)
{
    if (mx == 0 || my == 0 || mz == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    int16_t rx, ry, rz;
    LSM303D_Status_t status = LSM303D_ReadMagRaw(&rx, &ry, &rz);
    if (status != LSM303D_OK) {
        return status;
    }

    *mx = (float)rx * s_mag_sens;
    *my = (float)ry * s_mag_sens;
    *mz = (float)rz * s_mag_sens;

    return LSM303D_OK;
}

LSM303D_Status_t LSM303D_ReadMagMicroTesla(float *mx, float *my, float *mz)
{
    float gx, gy, gz;
    LSM303D_Status_t status = LSM303D_ReadMag(&gx, &gy, &gz);
    if (status != LSM303D_OK) {
        return status;
    }

    *mx = gx * GAUSS_TO_MICROTESLA;
    *my = gy * GAUSS_TO_MICROTESLA;
    *mz = gz * GAUSS_TO_MICROTESLA;

    return LSM303D_OK;
}

/*============================================================================*
 *                        TEMPERATURE READ FUNCTION                           *
 *============================================================================*/

LSM303D_Status_t LSM303D_ReadTemp(float *temp_c)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }
    if (temp_c == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    uint8_t buf[2];
    LSM303D_Status_t status = LSM303D_ReadBurst(LSM303D_REG_TEMP_OUT_L, buf, 2U);
    if (status != LSM303D_OK) {
        return status;
    }

    /* 12-bit two's complement value stored across 2 registers */
    int16_t raw_temp = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);

    /* Scale: 8 LSB / °C (0.125 °C/LSB) with factory offset */
    *temp_c = ((float)raw_temp * LSM303D_TEMP_SENS_DEG_C) + LSM303D_TEMP_OFFSET_DEG_C;

    return LSM303D_OK;
}

/*============================================================================*
 *                        CONSOLIDATED READ ALL                               *
 *============================================================================*/

LSM303D_Status_t LSM303D_ReadAll(LSM303D_Data_t *data)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }
    if (data == 0) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    /* 1. Read Accelerometer */
    LSM303D_Status_t st = LSM303D_ReadAccel(&data->accel_x, &data->accel_y, &data->accel_z);
    if (st != LSM303D_OK) {
        return st;
    }
    data->accel_mps2_x = data->accel_x * GRAVITY_EARTH_MPS2;
    data->accel_mps2_y = data->accel_y * GRAVITY_EARTH_MPS2;
    data->accel_mps2_z = data->accel_z * GRAVITY_EARTH_MPS2;

    /* 2. Read Magnetometer */
    st = LSM303D_ReadMag(&data->mag_x, &data->mag_y, &data->mag_z);
    if (st != LSM303D_OK) {
        return st;
    }
    data->mag_uT_x = data->mag_x * GAUSS_TO_MICROTESLA;
    data->mag_uT_y = data->mag_y * GAUSS_TO_MICROTESLA;
    data->mag_uT_z = data->mag_z * GAUSS_TO_MICROTESLA;

    /* 3. Read Temperature */
    st = LSM303D_ReadTemp(&data->temp_c);
    if (st != LSM303D_OK) {
        return st;
    }

    return LSM303D_OK;
}

/*============================================================================*
 *                        DATA STATUS CHECKS                                  *
 *============================================================================*/

bool LSM303D_IsAccelDataReady(void)
{
    uint8_t status = 0U;
    if (LSM303D_ReadReg(LSM303D_REG_STATUS_A, &status) != LSM303D_OK) {
        return false;
    }
    return ((status & LSM303D_STATUS_A_ZYXADA) != 0U);
}

bool LSM303D_IsMagDataReady(void)
{
    uint8_t status = 0U;
    if (LSM303D_ReadReg(LSM303D_REG_STATUS_M, &status) != LSM303D_OK) {
        return false;
    }
    return ((status & LSM303D_STATUS_M_ZYXDA) != 0U);
}

LSM303D_Status_t LSM303D_SetAccelFilterBW(LSM303D_AccelBW_t bw)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }

    uint8_t ctrl2 = 0U;
    LSM303D_Status_t st = LSM303D_ReadReg(LSM303D_REG_CTRL_REG2, &ctrl2);
    if (st != LSM303D_OK) {
        return st;
    }

    ctrl2 &= ~LSM303D_CTRL2_ABW_MASK;
    ctrl2 |= (uint8_t)((uint8_t)bw << LSM303D_CTRL2_ABW_POS);

    return LSM303D_WriteReg(LSM303D_REG_CTRL_REG2, ctrl2);
}

LSM303D_Status_t LSM303D_CalibrateAccelOffsets(uint16_t samples)
{
    if (!s_initialized) {
        return LSM303D_ERROR_NOT_INITIALIZED;
    }
    if (samples == 0U) {
        return LSM303D_ERROR_INVALID_PARAM;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    /* Reset existing offsets temporarily */
    s_accel_offset[0] = 0.0f;
    s_accel_offset[1] = 0.0f;
    s_accel_offset[2] = 0.0f;

    for (uint16_t i = 0U; i < samples; i++) {
        float ax, ay, az;
        LSM303D_Status_t st = LSM303D_ReadAccel(&ax, &ay, &az);
        if (st != LSM303D_OK) {
            return st;
        }
        sum_x += ax;
        sum_y += ay;
        sum_z += az;
        lsm303d_delay_ms(2U);
    }

    float inv_samples = 1.0f / (float)samples;
    s_accel_offset[0] = sum_x * inv_samples;
    s_accel_offset[1] = sum_y * inv_samples;
    /* Subtract nominal 1.0g gravity vector from Z axis offset */
    s_accel_offset[2] = (sum_z * inv_samples) - 1.0f;

    return LSM303D_OK;
}
