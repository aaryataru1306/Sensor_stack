#include "hmc5883l.h"
#include <math.h>

static const float gain_lsb[8] = {1370, 1090, 820, 660, 440, 390, 330, 230};

static int wr(hmc_dev_t *d, uint8_t reg, uint8_t val)
{
    return d->bus->write(d->bus->ctx, HMC5883L_ADDR, reg, &val, 1);
}

hmc_status_t hmc_set_gain(hmc_dev_t *d, hmc_gain_t gain)
{
    if (wr(d, HMC_REG_CONF_B, (uint8_t)gain)) return HMC_ERR_IO;
    d->lsb_per_gauss = gain_lsb[gain >> 5];
    return HMC_OK;
}

hmc_status_t hmc_set_mode(hmc_dev_t *d, hmc_mode_t mode)
{
    return wr(d, HMC_REG_MODE, (uint8_t)mode) ? HMC_ERR_IO : HMC_OK;
}

hmc_status_t hmc_init(hmc_dev_t *d, const hmc_bus_t *bus,
                      hmc_avg_t avg, hmc_odr_t odr, hmc_gain_t gain)
{
    uint8_t id[3];
    d->bus = bus;

    if (bus->read(bus->ctx, HMC5883L_ADDR, HMC_REG_ID_A, id, 3)) return HMC_ERR_IO;
    if (id[0] != 'H' || id[1] != '4' || id[2] != '3')            return HMC_ERR_ID;

    if (wr(d, HMC_REG_CONF_A, (uint8_t)(avg | odr))) return HMC_ERR_IO;
    if (hmc_set_gain(d, gain) != HMC_OK)             return HMC_ERR_IO;
    return hmc_set_mode(d, HMC_MODE_CONTINUOUS);
}

hmc_status_t hmc_data_ready(hmc_dev_t *d, uint8_t *ready)
{
    uint8_t s;
    if (d->bus->read(d->bus->ctx, HMC5883L_ADDR, HMC_REG_STATUS, &s, 1)) return HMC_ERR_IO;
    *ready = s & 0x01;
    return HMC_OK;
}

hmc_status_t hmc_read_raw(hmc_dev_t *d, hmc_raw_t *out)
{
    uint8_t b[6];
    if (d->bus->read(d->bus->ctx, HMC5883L_ADDR, HMC_REG_DATA_X_MSB, b, 6)) return HMC_ERR_IO;

    out->x = (int16_t)((b[0] << 8) | b[1]);
    out->z = (int16_t)((b[2] << 8) | b[3]);
    out->y = (int16_t)((b[4] << 8) | b[5]);

    if (out->x == -4096 || out->y == -4096 || out->z == -4096) return HMC_ERR_OVF;
    return HMC_OK;
}

hmc_status_t hmc_read_gauss(hmc_dev_t *d, hmc_gauss_t *out)
{
    hmc_raw_t r;
    hmc_status_t s = hmc_read_raw(d, &r);
    if (s != HMC_OK) return s;
    out->x = r.x / d->lsb_per_gauss;
    out->y = r.y / d->lsb_per_gauss;
    out->z = r.z / d->lsb_per_gauss;
    return HMC_OK;
}

float hmc_heading_deg(const hmc_gauss_t *g)
{
    float h = atan2f(g->y, g->x) * 180.0f / 3.14159265f;
    return (h < 0) ? h + 360.0f : h;
}
