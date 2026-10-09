#include "mpu6050.h"
#include <stddef.h>

#define REG_SMPLRT_DIV    0x19U
#define REG_CONFIG        0x1AU
#define REG_GYRO_CONFIG   0x1BU
#define REG_ACCEL_CONFIG  0x1CU
#define REG_INT_PIN_CFG   0x37U
#define REG_INT_ENABLE    0x38U
#define REG_INT_STATUS    0x3AU
#define REG_ACCEL_XOUT_H  0x3BU
#define REG_PWR_MGMT_1    0x6BU
#define REG_PWR_MGMT_2    0x6CU
#define REG_WHO_AM_I      0x75U

#define PWR1_DEVICE_RESET 0x80U
#define PWR1_SLEEP        0x40U
#define PWR1_CLKSEL_PLLX  0x01U
#define CONFIG_DLPF_MASK  0x07U
#define RANGE_MASK        0x18U
#define RANGE_SHIFT       3U
#define INT_PIN_RD_CLEAR  0x10U
#define INT_DATA_RDY      0x01U

#define RESET_DELAY_MS    100U
#define WAKE_DELAY_MS     10U
#define DEFAULT_RATE_HZ   100U
#define CALIB_MAX_SAMPLES 16384U
#define TEMP_SCALE        340.0f
#define TEMP_OFFSET_C     36.53f

static const float accel_lsb_table[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float gyro_lsb_table[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };

__attribute__((weak)) void mpu6050_delay_ms(uint32_t ms)
{
    volatile uint32_t n = (SystemCoreClock / 4000U) * ms;

    while (n--) {
        __NOP();
    }
}

static mpu6050_status_t bus_result(mpu6050_t *d, i2c_status_t st)
{
    d->last_i2c = st;
    return (st == I2C_OK) ? MPU6050_OK : MPU6050_ERR_BUS;
}

static mpu6050_status_t reg_write(mpu6050_t *d, uint8_t reg, uint8_t value)
{
    return bus_result(d, i2c_write_reg(d->bus, d->addr, reg, value));
}

static mpu6050_status_t reg_read(mpu6050_t *d, uint8_t reg, uint8_t *value)
{
    return bus_result(d, i2c_read_reg(d->bus, d->addr, reg, value));
}

static mpu6050_status_t reg_modify(mpu6050_t *d, uint8_t reg, uint8_t mask, uint8_t value)
{
    return bus_result(d, i2c_modify_reg(d->bus, d->addr, reg, mask, value));
}

static bool dev_ready(const mpu6050_t *d)
{
    return (d != NULL) && (d->bus != NULL) && d->initialized;
}

void mpu6050_get_default_config(mpu6050_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->accel_range = MPU6050_ACCEL_2G;
    cfg->gyro_range = MPU6050_GYRO_250DPS;
    cfg->dlpf = MPU6050_DLPF_44HZ;
    cfg->sample_rate_hz = DEFAULT_RATE_HZ;
}

mpu6050_status_t mpu6050_who_am_i(mpu6050_t *dev, uint8_t *id)
{
    if (dev == NULL || dev->bus == NULL || id == NULL) {
        return MPU6050_ERR_ARG;
    }
    return reg_read(dev, REG_WHO_AM_I, id);
}

mpu6050_status_t mpu6050_init(mpu6050_t *dev, i2c_t *bus, uint8_t addr, const mpu6050_config_t *cfg)
{
    mpu6050_config_t c;
    mpu6050_status_t st;
    uint8_t id = 0U;

    if (dev == NULL || bus == NULL ||
        (addr != MPU6050_ADDR_AD0_LOW && addr != MPU6050_ADDR_AD0_HIGH)) {
        return MPU6050_ERR_ARG;
    }

    if (cfg != NULL) {
        c = *cfg;
    } else {
        mpu6050_get_default_config(&c);
    }
    if ((uint32_t)c.accel_range > MPU6050_ACCEL_16G ||
        (uint32_t)c.gyro_range > MPU6050_GYRO_2000DPS ||
        (uint32_t)c.dlpf > MPU6050_DLPF_5HZ) {
        return MPU6050_ERR_ARG;
    }
    if (c.sample_rate_hz == 0U) {
        c.sample_rate_hz = DEFAULT_RATE_HZ;
    }

    dev->bus = bus;
    dev->addr = addr;
    dev->accel_range = c.accel_range;
    dev->gyro_range = c.gyro_range;
    dev->dlpf = c.dlpf;
    dev->sample_rate_hz = 0U;
    dev->sample_period_ms = 1U;
    dev->accel_lsb_per_g = accel_lsb_table[c.accel_range];
    dev->gyro_lsb_per_dps = gyro_lsb_table[c.gyro_range];
    dev->gyro_offset[0] = 0;
    dev->gyro_offset[1] = 0;
    dev->gyro_offset[2] = 0;
    dev->last_i2c = I2C_OK;
    dev->initialized = false;

    st = reg_read(dev, REG_WHO_AM_I, &id);
    if (st != MPU6050_OK) {
        return st;
    }
    if (id != MPU6050_WHO_AM_I_VALUE) {
        return MPU6050_ERR_ID;
    }

    st = reg_write(dev, REG_PWR_MGMT_1, PWR1_DEVICE_RESET);
    if (st != MPU6050_OK) {
        return st;
    }
    mpu6050_delay_ms(RESET_DELAY_MS);

    st = reg_write(dev, REG_PWR_MGMT_1, PWR1_CLKSEL_PLLX);
    if (st != MPU6050_OK) {
        return st;
    }
    mpu6050_delay_ms(WAKE_DELAY_MS);

    st = reg_write(dev, REG_PWR_MGMT_2, 0x00U);
    if (st != MPU6050_OK) {
        return st;
    }

    dev->initialized = true;

    st = mpu6050_set_dlpf(dev, c.dlpf);
    if (st == MPU6050_OK) {
        st = mpu6050_set_accel_range(dev, c.accel_range);
    }
    if (st == MPU6050_OK) {
        st = mpu6050_set_gyro_range(dev, c.gyro_range);
    }
    if (st == MPU6050_OK) {
        st = mpu6050_set_sample_rate(dev, c.sample_rate_hz);
    }
    if (st != MPU6050_OK) {
        dev->initialized = false;
    }
    return st;
}

mpu6050_status_t mpu6050_set_accel_range(mpu6050_t *dev, mpu6050_accel_range_t range)
{
    mpu6050_status_t st;

    if (!dev_ready(dev) || (uint32_t)range > MPU6050_ACCEL_16G) {
        return MPU6050_ERR_ARG;
    }
    st = reg_modify(dev, REG_ACCEL_CONFIG, RANGE_MASK, (uint8_t)((uint32_t)range << RANGE_SHIFT));
    if (st == MPU6050_OK) {
        dev->accel_range = range;
        dev->accel_lsb_per_g = accel_lsb_table[range];
    }
    return st;
}

mpu6050_status_t mpu6050_set_gyro_range(mpu6050_t *dev, mpu6050_gyro_range_t range)
{
    mpu6050_status_t st;

    if (!dev_ready(dev) || (uint32_t)range > MPU6050_GYRO_2000DPS) {
        return MPU6050_ERR_ARG;
    }
    st = reg_modify(dev, REG_GYRO_CONFIG, RANGE_MASK, (uint8_t)((uint32_t)range << RANGE_SHIFT));
    if (st == MPU6050_OK) {
        dev->gyro_range = range;
        dev->gyro_lsb_per_dps = gyro_lsb_table[range];
    }
    return st;
}

mpu6050_status_t mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t hz)
{
    mpu6050_status_t st;
    uint32_t base;
    uint32_t min_hz;
    uint32_t rate = hz;
    uint32_t div;

    if (!dev_ready(dev) || hz == 0U) {
        return MPU6050_ERR_ARG;
    }

    base = (dev->dlpf == MPU6050_DLPF_260HZ) ? 8000U : 1000U;
    min_hz = (base + 255U) / 256U;
    if (rate > base) {
        rate = base;
    }
    if (rate < min_hz) {
        rate = min_hz;
    }

    div = (base / rate) - 1U;
    st = reg_write(dev, REG_SMPLRT_DIV, (uint8_t)div);
    if (st != MPU6050_OK) {
        return st;
    }

    dev->sample_rate_hz = (uint16_t)(base / (div + 1U));
    dev->sample_period_ms = (1000U + dev->sample_rate_hz - 1U) / dev->sample_rate_hz;
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_set_dlpf(mpu6050_t *dev, mpu6050_dlpf_t dlpf)
{
    mpu6050_status_t st;

    if (!dev_ready(dev) || (uint32_t)dlpf > MPU6050_DLPF_5HZ) {
        return MPU6050_ERR_ARG;
    }
    st = reg_modify(dev, REG_CONFIG, CONFIG_DLPF_MASK, (uint8_t)dlpf);
    if (st != MPU6050_OK) {
        return st;
    }
    dev->dlpf = dlpf;
    if (dev->sample_rate_hz != 0U) {
        return mpu6050_set_sample_rate(dev, dev->sample_rate_hz);
    }
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t b[14];
    mpu6050_status_t st;

    if (!dev_ready(dev) || raw == NULL) {
        return MPU6050_ERR_ARG;
    }
    st = bus_result(dev, i2c_mem_read(dev->bus, dev->addr, REG_ACCEL_XOUT_H, 1U, b, (uint16_t)sizeof(b)));
    if (st != MPU6050_OK) {
        return st;
    }

    raw->ax   = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
    raw->ay   = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
    raw->az   = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
    raw->temp = (int16_t)(((uint16_t)b[6]  << 8) | b[7]);
    raw->gx   = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    raw->gy   = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    raw->gz   = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *data)
{
    mpu6050_raw_t r;
    mpu6050_status_t st;

    if (data == NULL) {
        return MPU6050_ERR_ARG;
    }
    st = mpu6050_read_raw(dev, &r);
    if (st != MPU6050_OK) {
        return st;
    }

    data->ax = (float)r.ax / dev->accel_lsb_per_g;
    data->ay = (float)r.ay / dev->accel_lsb_per_g;
    data->az = (float)r.az / dev->accel_lsb_per_g;
    data->gx = (float)((int32_t)r.gx - dev->gyro_offset[0]) / dev->gyro_lsb_per_dps;
    data->gy = (float)((int32_t)r.gy - dev->gyro_offset[1]) / dev->gyro_lsb_per_dps;
    data->gz = (float)((int32_t)r.gz - dev->gyro_offset[2]) / dev->gyro_lsb_per_dps;
    data->temp_c = ((float)r.temp / TEMP_SCALE) + TEMP_OFFSET_C;
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t samples)
{
    mpu6050_raw_t r;
    mpu6050_status_t st;
    int32_t sum[3] = { 0, 0, 0 };
    uint16_t i;

    if (!dev_ready(dev) || samples == 0U || samples > CALIB_MAX_SAMPLES) {
        return MPU6050_ERR_ARG;
    }

    for (i = 0U; i < samples; i++) {
        st = mpu6050_read_raw(dev, &r);
        if (st != MPU6050_OK) {
            return st;
        }
        sum[0] += r.gx;
        sum[1] += r.gy;
        sum[2] += r.gz;
        mpu6050_delay_ms(dev->sample_period_ms);
    }

    dev->gyro_offset[0] = (int16_t)(sum[0] / (int32_t)samples);
    dev->gyro_offset[1] = (int16_t)(sum[1] / (int32_t)samples);
    dev->gyro_offset[2] = (int16_t)(sum[2] / (int32_t)samples);
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_set_gyro_offset(mpu6050_t *dev, int16_t x, int16_t y, int16_t z)
{
    if (dev == NULL) {
        return MPU6050_ERR_ARG;
    }
    dev->gyro_offset[0] = x;
    dev->gyro_offset[1] = y;
    dev->gyro_offset[2] = z;
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_get_gyro_offset(const mpu6050_t *dev, int16_t offset[3])
{
    if (dev == NULL || offset == NULL) {
        return MPU6050_ERR_ARG;
    }
    offset[0] = dev->gyro_offset[0];
    offset[1] = dev->gyro_offset[1];
    offset[2] = dev->gyro_offset[2];
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_sleep(mpu6050_t *dev, bool enable)
{
    if (!dev_ready(dev)) {
        return MPU6050_ERR_ARG;
    }
    return reg_modify(dev, REG_PWR_MGMT_1, PWR1_SLEEP, enable ? PWR1_SLEEP : 0x00U);
}

mpu6050_status_t mpu6050_enable_data_ready_int(mpu6050_t *dev, bool enable)
{
    mpu6050_status_t st;

    if (!dev_ready(dev)) {
        return MPU6050_ERR_ARG;
    }
    if (enable) {
        st = reg_modify(dev, REG_INT_PIN_CFG, INT_PIN_RD_CLEAR, INT_PIN_RD_CLEAR);
        if (st != MPU6050_OK) {
            return st;
        }
    }
    return reg_modify(dev, REG_INT_ENABLE, INT_DATA_RDY, enable ? INT_DATA_RDY : 0x00U);
}

mpu6050_status_t mpu6050_data_ready(mpu6050_t *dev, bool *ready)
{
    uint8_t status = 0U;
    mpu6050_status_t st;

    if (!dev_ready(dev) || ready == NULL) {
        return MPU6050_ERR_ARG;
    }
    st = reg_read(dev, REG_INT_STATUS, &status);
    if (st != MPU6050_OK) {
        return st;
    }
    *ready = (status & INT_DATA_RDY) != 0U;
    return MPU6050_OK;
}