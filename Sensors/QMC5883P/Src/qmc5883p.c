#include "qmc5883p.h"
#include <stddef.h>
#include <math.h>

#define CTRL1_MODE_MASK     0x03U
#define CTRL1_ODR_SHIFT     2U
#define CTRL1_ODR_MASK      0x0CU
#define CTRL1_OSR1_SHIFT    4U
#define CTRL2_RANGE_MASK    0x03U
#define CTRL2_SOFT_RST      0x80U
#define STATUS_DRDY         0x01U
#define STATUS_OVFL         0x02U

#define RESET_DELAY_MS      10U
#define CALIB_MIN_SPAN      50.0f
#define RAD_TO_DEG          57.2957795f

static const float lsb_table[4] = { 1000.0f, 2500.0f, 3750.0f, 15000.0f };

/* Output period per ODR setting (ms) */
static const uint16_t period_ms_table[4] = { 100U, 20U, 10U, 5U };

__attribute__((weak)) void qmc5883p_delay_ms(uint32_t ms)
{
    volatile uint32_t n = (SystemCoreClock / 4000U) * ms;

    while (n--) {
        __NOP();
    }
}

static qmc5883p_status_t bus_result(qmc5883p_t *d, i2c_status_t st)
{
    d->last_i2c = st;
    return (st == I2C_OK) ? QMC5883P_OK : QMC5883P_ERR_BUS;
}

static qmc5883p_status_t reg_write(qmc5883p_t *d, uint8_t reg, uint8_t value)
{
    return bus_result(d, i2c_write_reg(d->bus, d->addr, reg, value));
}

static qmc5883p_status_t reg_read(qmc5883p_t *d, uint8_t reg, uint8_t *value)
{
    return bus_result(d, i2c_read_reg(d->bus, d->addr, reg, value));
}

static qmc5883p_status_t reg_modify(qmc5883p_t *d, uint8_t reg, uint8_t mask, uint8_t value)
{
    return bus_result(d, i2c_modify_reg(d->bus, d->addr, reg, mask, value));
}

static bool dev_ready(const qmc5883p_t *d)
{
    return (d != NULL) && (d->bus != NULL) && d->initialized;
}

static qmc5883p_status_t apply_config(qmc5883p_t *d)
{
    qmc5883p_status_t st;
    uint8_t ctrl1 = (uint8_t)(((uint32_t)d->dsr << 6) |
                              ((uint32_t)d->osr << CTRL1_OSR1_SHIFT) |
                              ((uint32_t)d->odr << CTRL1_ODR_SHIFT) |
                              (uint32_t)d->mode);
    uint8_t ctrl2 = (uint8_t)d->range;      /* set/reset left at default (on) */

    st = reg_write(d, QMC5883P_REG_CTRL2, ctrl2);
    if (st != QMC5883P_OK) {
        return st;
    }
    return reg_write(d, QMC5883P_REG_CTRL1, ctrl1);
}

static qmc5883p_status_t wait_ready(qmc5883p_t *d, uint8_t *status)
{
    qmc5883p_status_t st;
    uint32_t left = (3U * period_ms_table[d->odr]) + 20U;
    uint8_t s = 0U;

    for (;;) {
        st = reg_read(d, QMC5883P_REG_STATUS, &s);
        if (st != QMC5883P_OK) {
            return st;
        }
        if (s & STATUS_DRDY) {
            *status = s;
            return QMC5883P_OK;
        }
        if (left == 0U) {
            return QMC5883P_ERR_TIMEOUT;
        }
        left--;
        qmc5883p_delay_ms(1U);
    }
}

void qmc5883p_get_default_config(qmc5883p_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->range = QMC5883P_RANGE_8G;
    cfg->odr   = QMC5883P_ODR_50HZ;
    cfg->osr   = QMC5883P_OSR_8;
    cfg->dsr   = QMC5883P_DSR_1;
    cfg->mode  = QMC5883P_MODE_CONTINUOUS;
}

qmc5883p_status_t qmc5883p_read_id(qmc5883p_t *dev, uint8_t *id)
{
    if (dev == NULL || dev->bus == NULL || id == NULL) {
        return QMC5883P_ERR_ARG;
    }
    return reg_read(dev, QMC5883P_REG_CHIP_ID, id);
}

qmc5883p_status_t qmc5883p_init(qmc5883p_t *dev, i2c_t *bus, const qmc5883p_config_t *cfg)
{
    qmc5883p_config_t c;
    qmc5883p_status_t st;
    uint8_t id = 0U;

    if (dev == NULL || bus == NULL) {
        return QMC5883P_ERR_ARG;
    }
    if (cfg != NULL) {
        c = *cfg;
    } else {
        qmc5883p_get_default_config(&c);
    }
    if ((uint32_t)c.range > QMC5883P_RANGE_2G ||
        (uint32_t)c.odr > QMC5883P_ODR_200HZ ||
        (uint32_t)c.osr > QMC5883P_OSR_1 ||
        (uint32_t)c.dsr > QMC5883P_DSR_8 ||
        (c.mode != QMC5883P_MODE_SUSPEND && c.mode != QMC5883P_MODE_CONTINUOUS)) {
        return QMC5883P_ERR_ARG;
    }

    dev->bus = bus;
    dev->addr = QMC5883P_I2C_ADDR;
    dev->range = c.range;
    dev->odr = c.odr;
    dev->osr = c.osr;
    dev->dsr = c.dsr;
    dev->mode = c.mode;
    dev->lsb_per_gauss = lsb_table[c.range];
    dev->offset[0] = 0.0f;
    dev->offset[1] = 0.0f;
    dev->offset[2] = 0.0f;
    dev->scale[0] = 1.0f;
    dev->scale[1] = 1.0f;
    dev->scale[2] = 1.0f;
    dev->declination_deg = 0.0f;
    dev->last_i2c = I2C_OK;
    dev->initialized = false;

    st = qmc5883p_read_id(dev, &id);
    if (st != QMC5883P_OK) {
        return st;
    }
    if (id != QMC5883P_CHIP_ID_VALUE) {
        return QMC5883P_ERR_ID;
    }

    st = reg_write(dev, QMC5883P_REG_CTRL2, CTRL2_SOFT_RST);
    if (st != QMC5883P_OK) {
        return st;
    }
    qmc5883p_delay_ms(RESET_DELAY_MS);

    st = apply_config(dev);
    if (st != QMC5883P_OK) {
        return st;
    }
    qmc5883p_delay_ms(RESET_DELAY_MS);

    dev->initialized = true;
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_set_range(qmc5883p_t *dev, qmc5883p_range_t range)
{
    qmc5883p_status_t st;

    if (!dev_ready(dev) || (uint32_t)range > QMC5883P_RANGE_2G) {
        return QMC5883P_ERR_ARG;
    }
    st = reg_modify(dev, QMC5883P_REG_CTRL2, CTRL2_RANGE_MASK, (uint8_t)range);
    if (st == QMC5883P_OK) {
        dev->range = range;
        dev->lsb_per_gauss = lsb_table[range];
    }
    return st;
}

qmc5883p_status_t qmc5883p_set_odr(qmc5883p_t *dev, qmc5883p_odr_t odr)
{
    qmc5883p_status_t st;

    if (!dev_ready(dev) || (uint32_t)odr > QMC5883P_ODR_200HZ) {
        return QMC5883P_ERR_ARG;
    }
    st = reg_modify(dev, QMC5883P_REG_CTRL1, CTRL1_ODR_MASK, (uint8_t)((uint32_t)odr << CTRL1_ODR_SHIFT));
    if (st == QMC5883P_OK) {
        dev->odr = odr;
    }
    return st;
}

qmc5883p_status_t qmc5883p_set_mode(qmc5883p_t *dev, qmc5883p_mode_t mode)
{
    qmc5883p_status_t st;

    if (!dev_ready(dev) ||
        (mode != QMC5883P_MODE_SUSPEND && mode != QMC5883P_MODE_CONTINUOUS)) {
        return QMC5883P_ERR_ARG;
    }
    st = reg_modify(dev, QMC5883P_REG_CTRL1, CTRL1_MODE_MASK, (uint8_t)mode);
    if (st == QMC5883P_OK) {
        dev->mode = mode;
    }
    return st;
}

qmc5883p_status_t qmc5883p_data_ready(qmc5883p_t *dev, bool *ready)
{
    uint8_t s = 0U;
    qmc5883p_status_t st;

    if (!dev_ready(dev) || ready == NULL) {
        return QMC5883P_ERR_ARG;
    }
    st = reg_read(dev, QMC5883P_REG_STATUS, &s);
    if (st != QMC5883P_OK) {
        return st;
    }
    *ready = (s & STATUS_DRDY) != 0U;
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_read_raw(qmc5883p_t *dev, qmc5883p_raw_t *raw)
{
    qmc5883p_status_t st;
    uint8_t b[6];
    uint8_t s = 0U;

    if (!dev_ready(dev) || raw == NULL || dev->mode != QMC5883P_MODE_CONTINUOUS) {
        return QMC5883P_ERR_ARG;
    }
    st = wait_ready(dev, &s);
    if (st != QMC5883P_OK) {
        return st;
    }
    st = bus_result(dev, i2c_mem_read(dev->bus, dev->addr, QMC5883P_REG_DATA_X_L, 1U, b, (uint16_t)sizeof(b)));
    if (st != QMC5883P_OK) {
        return st;
    }

    /* Little-endian, order X, Y, Z */
    raw->x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    raw->y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    raw->z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);

    return (s & STATUS_OVFL) ? QMC5883P_ERR_OVERFLOW : QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_read(qmc5883p_t *dev, qmc5883p_data_t *data)
{
    qmc5883p_raw_t r;
    qmc5883p_status_t st;
    float cx;
    float cy;
    float cz;
    float h;

    if (data == NULL) {
        return QMC5883P_ERR_ARG;
    }
    st = qmc5883p_read_raw(dev, &r);
    if (st != QMC5883P_OK) {
        return st;
    }

    cx = ((float)r.x - dev->offset[0]) * dev->scale[0];
    cy = ((float)r.y - dev->offset[1]) * dev->scale[1];
    cz = ((float)r.z - dev->offset[2]) * dev->scale[2];

    data->x = cx / dev->lsb_per_gauss;
    data->y = cy / dev->lsb_per_gauss;
    data->z = cz / dev->lsb_per_gauss;
    data->magnitude_g = sqrtf((data->x * data->x) + (data->y * data->y) + (data->z * data->z));

    h = (atan2f(cy, cx) * RAD_TO_DEG) + dev->declination_deg;
    while (h < 0.0f) {
        h += 360.0f;
    }
    while (h >= 360.0f) {
        h -= 360.0f;
    }
    data->heading_deg = h;
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_calibrate(qmc5883p_t *dev, uint16_t samples)
{
    qmc5883p_raw_t r;
    qmc5883p_status_t st;
    float mn[3] = {  32767.0f,  32767.0f,  32767.0f };
    float mx[3] = { -32768.0f, -32768.0f, -32768.0f };
    float v[3];
    float span[3];
    float avg;
    uint16_t i;
    uint8_t k;

    if (!dev_ready(dev) || samples < 2U) {
        return QMC5883P_ERR_ARG;
    }

    for (i = 0U; i < samples; i++) {
        st = qmc5883p_read_raw(dev, &r);
        if (st == QMC5883P_ERR_OVERFLOW) {
            continue;                   /* skip saturated samples */
        }
        if (st != QMC5883P_OK) {
            return st;
        }
        v[0] = (float)r.x;
        v[1] = (float)r.y;
        v[2] = (float)r.z;
        for (k = 0U; k < 3U; k++) {
            if (v[k] < mn[k]) { mn[k] = v[k]; }
            if (v[k] > mx[k]) { mx[k] = v[k]; }
        }
    }

    for (k = 0U; k < 3U; k++) {
        span[k] = (mx[k] - mn[k]) * 0.5f;
        if (span[k] < CALIB_MIN_SPAN) {
            return QMC5883P_ERR_CALIB;  /* not rotated enough */
        }
    }
    avg = (span[0] + span[1] + span[2]) / 3.0f;
    for (k = 0U; k < 3U; k++) {
        dev->offset[k] = (mx[k] + mn[k]) * 0.5f;
        dev->scale[k] = avg / span[k];
    }
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_set_calibration(qmc5883p_t *dev, const float offset[3], const float scale[3])
{
    uint8_t k;

    if (dev == NULL || offset == NULL || scale == NULL) {
        return QMC5883P_ERR_ARG;
    }
    for (k = 0U; k < 3U; k++) {
        dev->offset[k] = offset[k];
        dev->scale[k] = scale[k];
    }
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_get_calibration(const qmc5883p_t *dev, float offset[3], float scale[3])
{
    uint8_t k;

    if (dev == NULL || offset == NULL || scale == NULL) {
        return QMC5883P_ERR_ARG;
    }
    for (k = 0U; k < 3U; k++) {
        offset[k] = dev->offset[k];
        scale[k] = dev->scale[k];
    }
    return QMC5883P_OK;
}

qmc5883p_status_t qmc5883p_set_declination(qmc5883p_t *dev, float deg)
{
    if (dev == NULL) {
        return QMC5883P_ERR_ARG;
    }
    dev->declination_deg = deg;
    return QMC5883P_OK;
}