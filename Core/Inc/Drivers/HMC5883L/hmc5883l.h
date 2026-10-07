#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdint.h>

#define HMC5883L_ADDR      0x1E

#define HMC_REG_CONF_A     0x00
#define HMC_REG_CONF_B     0x01
#define HMC_REG_MODE       0x02
#define HMC_REG_DATA_X_MSB 0x03
#define HMC_REG_STATUS     0x09
#define HMC_REG_ID_A       0x0A

typedef enum {
    HMC_GAIN_0_88 = 0x00, HMC_GAIN_1_3 = 0x20, HMC_GAIN_1_9 = 0x40,
    HMC_GAIN_2_5  = 0x60, HMC_GAIN_4_0 = 0x80, HMC_GAIN_4_7 = 0xA0,
    HMC_GAIN_5_6  = 0xC0, HMC_GAIN_8_1 = 0xE0
} hmc_gain_t;

typedef enum {
    HMC_ODR_0_75 = 0x00, HMC_ODR_1_5 = 0x04, HMC_ODR_3 = 0x08,
    HMC_ODR_7_5  = 0x0C, HMC_ODR_15  = 0x10, HMC_ODR_30 = 0x14,
    HMC_ODR_75   = 0x18
} hmc_odr_t;

typedef enum {
    HMC_AVG_1 = 0x00, HMC_AVG_2 = 0x20, HMC_AVG_4 = 0x40, HMC_AVG_8 = 0x60
} hmc_avg_t;

typedef enum { HMC_MODE_CONTINUOUS = 0, HMC_MODE_SINGLE = 1, HMC_MODE_IDLE = 2 } hmc_mode_t;

typedef struct { int16_t x, y, z; } hmc_raw_t;
typedef struct { float x, y, z; } hmc_gauss_t;

typedef enum { HMC_OK = 0, HMC_ERR_IO = -1, HMC_ERR_ID = -2, HMC_ERR_OVF = -3 } hmc_status_t;

typedef struct {
    void *ctx;
    int (*write)(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
    int (*read) (void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
} hmc_bus_t;

typedef struct {
    const hmc_bus_t *bus;
    float lsb_per_gauss;
} hmc_dev_t;

hmc_status_t hmc_init(hmc_dev_t *d, const hmc_bus_t *bus,
                      hmc_avg_t avg, hmc_odr_t odr, hmc_gain_t gain);
hmc_status_t hmc_set_mode(hmc_dev_t *d, hmc_mode_t mode);
hmc_status_t hmc_set_gain(hmc_dev_t *d, hmc_gain_t gain);
hmc_status_t hmc_data_ready(hmc_dev_t *d, uint8_t *ready);
hmc_status_t hmc_read_raw(hmc_dev_t *d, hmc_raw_t *out);
hmc_status_t hmc_read_gauss(hmc_dev_t *d, hmc_gauss_t *out);
float        hmc_heading_deg(const hmc_gauss_t *g);

#endif
