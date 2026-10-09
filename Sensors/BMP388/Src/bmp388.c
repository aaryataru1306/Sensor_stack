#include <stddef.h>
#include <math.h>
#include "bmp388.h"

#define REG_STATUS          0x03U
#define REG_DATA_START      0x04U
#define REG_CALIB_START     0x31U
#define REG_CALIB_LENGTH    21U
#define REG_OSR             0x1CU
#define REG_ODR             0x1DU
#define REG_PWR_CTRL        0x1BU
#define REG_CMD             0x7EU

#define SOFT_RESET_COMMAND  0xB6U

#define BMP388_OSR_NO_OVERSAMPLING   0x00U
#define BMP388_ODR_100_HZ            0x03U
#define BMP388_NORMAL_MODE           0x33U

#define SEA_LEVEL_PRESSURE_PA        101325.0f

typedef struct
{
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    float t_lin;
    uint8_t valid;
} BMP388_Calibration;

static i2c_t *g_bmp388_i2c = NULL;
static uint8_t g_bmp388_addr = BMP388_I2C_ADDR_PRIM;
static BMP388_Calibration calib = {0};

static void soft_delay(uint32_t count)
{
    for (volatile uint32_t i = 0U; i < count; i++)
    {
        __asm volatile ("nop");
    }
}

static int read_registers(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    if ((g_bmp388_i2c == NULL) || (buffer == NULL) || (length == 0U))
    {
        return BMP388_ERROR;
    }

    if (i2c_mem_read(g_bmp388_i2c, g_bmp388_addr, reg, 1U, buffer, length) != I2C_OK)
    {
        return BMP388_ERROR;
    }

    return BMP388_OK;
}

static int write_register(uint8_t reg, uint8_t value)
{
    if (g_bmp388_i2c == NULL)
    {
        return BMP388_ERROR;
    }

    if (i2c_write_reg(g_bmp388_i2c, g_bmp388_addr, reg, value) != I2C_OK)
    {
        return BMP388_ERROR;
    }

    return BMP388_OK;
}

/* ---------------------------------------------------------
 * Bosch BMP388 Calibration Unpacking
 * --------------------------------------------------------- */

static int read_calibration(void)
{
    uint8_t c[REG_CALIB_LENGTH];

    if (read_registers(REG_CALIB_START, c, REG_CALIB_LENGTH) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    /* Unpack raw uint16/int16 trimming registers */
    uint16_t n_t1 = (uint16_t)(((uint16_t)c[1] << 8) | c[0]);
    uint16_t n_t2 = (uint16_t)(((uint16_t)c[3] << 8) | c[2]);
    int8_t   n_t3 = (int8_t)c[4];

    int16_t  n_p1 = (int16_t)(((uint16_t)c[6] << 8) | c[5]);
    int16_t  n_p2 = (int16_t)(((uint16_t)c[8] << 8) | c[7]);
    int8_t   n_p3 = (int8_t)c[9];
    int8_t   n_p4 = (int8_t)c[10];
    uint16_t n_p5 = (uint16_t)(((uint16_t)c[12] << 8) | c[11]);
    uint16_t n_p6 = (uint16_t)(((uint16_t)c[14] << 8) | c[13]);
    int8_t   n_p7 = (int8_t)c[15];
    int8_t   n_p8 = (int8_t)c[16];
    int16_t  n_p9 = (int16_t)(((uint16_t)c[18] << 8) | c[17]);
    int8_t   n_p10 = (int8_t)c[19];
    int8_t   n_p11 = (int8_t)c[20];

    /* Convert to floating point scaling factors according to datasheet */
    calib.t1 = (float)n_t1 / 0.00390625f;                    /* / 2^-8  */
    calib.t2 = (float)n_t2 / 1073741824.0f;                  /* / 2^30  */
    calib.t3 = (float)n_t3 / 281474976710656.0f;             /* / 2^48  */

    calib.p1 = ((float)n_p1 - 16384.0f) / 1048576.0f;        /* / 2^20  */
    calib.p2 = ((float)n_p2 - 16384.0f) / 536870912.0f;       /* / 2^29  */
    calib.p3 = (float)n_p3 / 4294967296.0f;                  /* / 2^32  */
    calib.p4 = (float)n_p4 / 137438953472.0f;                /* / 2^37  */
    calib.p5 = (float)n_p5 / 0.125f;                         /* / 2^-3  */
    calib.p6 = (float)n_p6 / 64.0f;                          /* / 2^6   */
    calib.p7 = (float)n_p7 / 256.0f;                         /* / 2^8   */
    calib.p8 = (float)n_p8 / 32768.0f;                       /* / 2^15  */
    calib.p9 = (float)n_p9 / 281474976710656.0f;             /* / 2^48  */
    calib.p10 = (float)n_p10 / 281474976710656.0f;           /* / 2^48  */
    calib.p11 = (float)n_p11 / 36893488147419103232.0f;      /* / 2^65  */

    calib.valid = 1U;
    return BMP388_OK;
}

int BMP388_ReadChipID(uint8_t *chip_id)
{
    if (chip_id == NULL)
    {
        return BMP388_ERROR;
    }

    return read_registers(BMP388_REG_CHIP_ID, chip_id, 1U);
}

int BMP388_Init(i2c_t *i2c_dev, uint8_t i2c_addr)
{
    uint8_t chip_id = 0U;

    if (i2c_dev == NULL)
    {
        return BMP388_ERROR;
    }

    g_bmp388_i2c = i2c_dev;
    g_bmp388_addr = i2c_addr;
    calib.valid = 0U;

    (void)write_register(REG_CMD, SOFT_RESET_COMMAND);
    soft_delay(160000U);

    if (BMP388_ReadChipID(&chip_id) != BMP388_OK || chip_id != BMP388_CHIP_ID)
    {
        return BMP388_INVALID_ID;
    }

    if (read_calibration() != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    if (write_register(REG_OSR, BMP388_OSR_NO_OVERSAMPLING) != BMP388_OK)
    {
        return BMP388_ERROR;
    }
    soft_delay(20000U);

    if (write_register(REG_ODR, BMP388_ODR_100_HZ) != BMP388_OK)
    {
        return BMP388_ERROR;
    }
    soft_delay(20000U);

    if (write_register(REG_PWR_CTRL, BMP388_NORMAL_MODE) != BMP388_OK)
    {
        return BMP388_ERROR;
    }
    soft_delay(50000U);

    return BMP388_OK;
}

/* ---------------------------------------------------------
 * Compensation Formulas (Bosch API Compliant)
 * --------------------------------------------------------- */

static float compensate_temperature(uint32_t raw_temp)
{
    float uncomp_temp = (float)raw_temp;
    float partial_data1 = uncomp_temp - calib.t1;
    float partial_data2 = partial_data1 * calib.t2;

    calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib.t3;

    return calib.t_lin;
}

static float compensate_pressure(uint32_t raw_press)
{
    float uncomp_press = (float)raw_press;
    float t = calib.t_lin;

    float partial_data1 = calib.p6 * t;
    float partial_data2 = calib.p7 * (t * t);
    float partial_data3 = calib.p8 * (t * t * t);
    float partial_out1 = calib.p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib.p2 * t;
    partial_data2 = calib.p3 * (t * t);
    partial_data3 = calib.p4 * (t * t * t);
    float partial_out2 = uncomp_press * (calib.p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = calib.p9 + calib.p10 * t;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + (uncomp_press * uncomp_press * uncomp_press) * calib.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

int BMP388_ReadData(BMP388_Data *data)
{
    uint8_t raw[6];

    if ((data == NULL) || (calib.valid == 0U))
    {
        return BMP388_ERROR;
    }

    if (read_registers(REG_DATA_START, raw, 6U) != BMP388_OK)
    {
        return BMP388_ERROR;
    }

    uint32_t raw_press = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
    uint32_t raw_temp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

    if (raw_press == 0U || raw_temp == 0U)
    {
        return BMP388_ERROR;
    }

    data->temperature_c = compensate_temperature(raw_temp);
    data->pressure_pa   = compensate_pressure(raw_press);

    if (data->pressure_pa > 0.0f)
    {
        data->altitude_m = 44330.0f * (1.0f - powf(data->pressure_pa / SEA_LEVEL_PRESSURE_PA, 0.1903f));
    }
    else
    {
        data->altitude_m = 0.0f;
    }

    return BMP388_OK;
}