#include "bmp388.h"
#include "spi.h"
#include <stdint.h>

/* BMP388 register addresses */
#define REG_STATUS          0x03U
#define REG_DATA_START      0x04U
#define REG_CALIB_START     0x31U
#define REG_CALIB_LENGTH    21U
#define REG_OSR             0x1CU
#define REG_ODR             0x1DU
#define REG_PWR_CTRL        0x1BU
#define REG_CMD             0x7EU

#define SOFT_RESET_COMMAND  0xB6U

/* Data-ready flags */
#define BMP388_PRESS_READY  (1U << 5)
#define BMP388_TEMP_READY   (1U << 6)
#define BMP388_DATA_READY   (BMP388_PRESS_READY | BMP388_TEMP_READY)

/* Sensor configuration */
#define BMP388_OSR_X2_TEMP_X4_PRESS  0x28U
#define BMP388_ODR_50_HZ             0x02U
#define BMP388_NORMAL_MODE           0x33U

#define BMP388_POLL_LIMIT   100000U

typedef struct
{
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6;
    float p7, p8, p9, p10, p11;
    float t_lin;
    uint8_t valid;
} BMP388_Calibration;

static BMP388_Calibration calib = {0};

/* ---------------------------------------------------------
 * SPI register access using the team's shared SPI driver
 * --------------------------------------------------------- */

static int read_registers(uint8_t reg,
                          uint8_t *buffer,
                          uint16_t length)
{
    if ((buffer == 0U) || (length == 0U))
    {
        return BMP388_ERROR;
    }

    SPI_CS_Select(SPI1_CS_BMP388_PIN);

    /* BMP388 SPI read: set bit 7 of the register address. */
    (void)SPI1_TransferByte((uint8_t)(reg | 0x80U));

    /* BMP388 SPI read protocol requires a dummy byte. */
    (void)SPI1_TransferByte(0x00U);

    for (uint16_t i = 0U; i < length; i++)
    {
        buffer[i] = SPI1_TransferByte(0x00U);
    }

    SPI_CS_Deselect(SPI1_CS_BMP388_PIN);

    return BMP388_OK;
}

static int write_register(uint8_t reg, uint8_t value)
{
    SPI_CS_Select(SPI1_CS_BMP388_PIN);

    /* BMP388 SPI write: bit 7 cleared. */
    (void)SPI1_TransferByte((uint8_t)(reg & 0x7FU));
    (void)SPI1_TransferByte(value);

    SPI_CS_Deselect(SPI1_CS_BMP388_PIN);

    return BMP388_OK;
}

/* ---------------------------------------------------------
 * Calibration data conversion
 * --------------------------------------------------------- */

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] |
                     ((uint16_t)p[1] << 8));
}

static int16_t read_s16_le(const uint8_t *p)
{
    uint16_t raw = read_u16_le(p);

    if (raw <= 32767U)
    {
        return (int16_t)raw;
    }

    return (int16_t)(-1 - (int32_t)(65535U - raw));
}

static int read_calibration(void)
{
    uint8_t c[REG_CALIB_LENGTH];

    if (read_registers(REG_CALIB_START,
                       c,
                       REG_CALIB_LENGTH) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    /* Temperature coefficients */
    calib.t1 = (float)read_u16_le(&c[0]) / 256.0f;
    calib.t2 = (float)read_u16_le(&c[2]) / 1073741824.0f;
    calib.t3 = (float)(int8_t)c[4] / 281474976710656.0f;

    /* Pressure coefficients */
    calib.p1 = ((float)read_s16_le(&c[5]) - 16384.0f)
               / 1048576.0f;

    calib.p2 = ((float)read_s16_le(&c[7]) - 16384.0f)
               / 536870912.0f;

    calib.p3 = (float)(int8_t)c[9] / 2147483648.0f;
    calib.p4 = (float)(int8_t)c[10] / 137438953472.0f;
    calib.p5 = (float)read_u16_le(&c[11]) / 8.0f;
    calib.p6 = (float)read_u16_le(&c[13]) / 64.0f;
    calib.p7 = (float)(int8_t)c[15] / 256.0f;
    calib.p8 = (float)(int8_t)c[16] / 32768.0f;

    calib.p9 = (float)read_s16_le(&c[17])
               / 281474976710656.0f;

    calib.p10 = (float)(int8_t)c[19]
                / 281474976710656.0f;

    calib.p11 = (float)(int8_t)c[20]
                / 36893488147419103232.0f;

    calib.valid = 1U;

    return BMP388_OK;
}

/* ---------------------------------------------------------
 * Chip identification
 * --------------------------------------------------------- */

int BMP388_ReadChipID(uint8_t *chip_id)
{
    if (chip_id == 0U)
    {
        return BMP388_ERROR;
    }

    return read_registers(BMP388_REG_CHIP_ID, chip_id, 1U);
}

/* ---------------------------------------------------------
 * Sensor initialization
 * --------------------------------------------------------- */

int BMP388_Init(void)
{
    uint8_t chip_id = 0U;

    calib.valid = 0U;

    /* Issue soft reset */
    if (write_register(REG_CMD, SOFT_RESET_COMMAND) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    /* Temporary startup delay; not a calibrated time delay. */
    for (volatile uint32_t i = 0U; i < 100000U; i++)
    {
        __asm volatile ("nop");
    }

    /* Verify device identity */
    if (BMP388_ReadChipID(&chip_id) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    if (chip_id != BMP388_CHIP_ID)
    {
        return BMP388_INVALID_ID;
    }

    /* Read factory calibration coefficients */
    if (read_calibration() != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    /* Temperature oversampling x2; pressure oversampling x4 */
    if (write_register(REG_OSR,
                       BMP388_OSR_X2_TEMP_X4_PRESS) != BMP388_OK)
    {
        calib.valid = 0U;
        return BMP388_ERROR;
    }

    /* Configure output data rate */
    if (write_register(REG_ODR, BMP388_ODR_50_HZ) != BMP388_OK)
    {
        calib.valid = 0U;
        return BMP388_ERROR;
    }

    /* Enable pressure and temperature; normal mode */
    if (write_register(REG_PWR_CTRL, BMP388_NORMAL_MODE) != BMP388_OK)
    {
        calib.valid = 0U;
        return BMP388_ERROR;
    }

    return BMP388_OK;
}

/* ---------------------------------------------------------
 * Temperature compensation
 * --------------------------------------------------------- */

static float compensate_temperature(uint32_t raw_temp)
{
    float d1 = (float)raw_temp - calib.t1;
    float d2 = d1 * calib.t2;

    calib.t_lin = d2 + (d1 * d1) * calib.t3;

    return calib.t_lin;
}

/* ---------------------------------------------------------
 * Pressure compensation
 * --------------------------------------------------------- */

static float compensate_pressure(uint32_t raw_press)
{
    float t = calib.t_lin;

    float d1 = calib.p6 * t;
    float d2 = calib.p7 * t * t;
    float d3 = calib.p8 * t * t * t;

    float out1 = calib.p5 + d1 + d2 + d3;

    d1 = calib.p2 * t;
    d2 = calib.p3 * t * t;
    d3 = calib.p4 * t * t * t;

    float out2 = (float)raw_press *
                 (calib.p1 + d1 + d2 + d3);

    float raw_press_squared = (float)raw_press * (float)raw_press;

    d1 = raw_press_squared;
    d2 = calib.p9 + calib.p10 * t;
    d3 = d1 * d2;

    float d4 = d3 +
               raw_press_squared *
               (float)raw_press * calib.p11;

    return out1 + out2 + d4;
}

/* ---------------------------------------------------------
 * Read compensated temperature and pressure
 * --------------------------------------------------------- */

int BMP388_ReadData(BMP388_Data *data)
{
    uint8_t status = 0U;
    uint8_t raw[6];
    uint32_t timeout = BMP388_POLL_LIMIT;

    if ((data == 0U) || (calib.valid == 0U))
    {
        return BMP388_ERROR;
    }

    /* Wait for both temperature and pressure data */
    while (timeout > 0U)
    {
        if (read_registers(REG_STATUS, &status, 1U) != BMP388_OK)
        {
            return BMP388_ERROR;
        }

        if ((status & BMP388_DATA_READY) == BMP388_DATA_READY)
        {
            break;
        }

        timeout--;
    }

    if (timeout == 0U)
    {
        return BMP388_ERROR;
    }

    /* Burst-read pressure and temperature registers 0x04–0x09 */
    if (read_registers(REG_DATA_START, raw, 6U) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    uint32_t raw_press =
        (uint32_t)raw[0] |
        ((uint32_t)raw[1] << 8) |
        ((uint32_t)raw[2] << 16);

    uint32_t raw_temp =
        (uint32_t)raw[3] |
        ((uint32_t)raw[4] << 8) |
        ((uint32_t)raw[5] << 16);

    data->temperature_c = compensate_temperature(raw_temp);
    data->pressure_pa = compensate_pressure(raw_press);

    return BMP388_OK;
}
