#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit I2C address */
#define QMC5883P_I2C_ADDR       0x2CU

/* Registers */
#define QMC5883P_REG_CHIP_ID    0x00U   /* expect 0x80 */
#define QMC5883P_REG_DATA_X_L   0x01U   /* X_L X_H Y_L Y_H Z_L Z_H */
#define QMC5883P_REG_STATUS     0x09U   /* bit0 DRDY, bit1 OVFL */
#define QMC5883P_REG_CTRL1      0x0AU   /* mode, ODR, OSR1, OSR2 */
#define QMC5883P_REG_CTRL2      0x0BU   /* range, set/reset, soft reset */

#define QMC5883P_CHIP_ID_VALUE  0x80U

typedef enum {
    QMC5883P_OK = 0,
    QMC5883P_ERR_ARG,
    QMC5883P_ERR_BUS,
    QMC5883P_ERR_ID,
    QMC5883P_ERR_TIMEOUT,
    QMC5883P_ERR_OVERFLOW,
    QMC5883P_ERR_CALIB
} qmc5883p_status_t;

typedef enum {
    QMC5883P_RANGE_30G = 0,     /* 1000 LSB/G  */
    QMC5883P_RANGE_12G,         /* 2500 LSB/G  */
    QMC5883P_RANGE_8G,          /* 3750 LSB/G  */
    QMC5883P_RANGE_2G           /* 15000 LSB/G */
} qmc5883p_range_t;

typedef enum {
    QMC5883P_ODR_10HZ = 0,
    QMC5883P_ODR_50HZ,
    QMC5883P_ODR_100HZ,
    QMC5883P_ODR_200HZ
} qmc5883p_odr_t;

typedef enum {
    QMC5883P_OSR_8 = 0,         /* over-sample ratio (OSR1) */
    QMC5883P_OSR_4,
    QMC5883P_OSR_2,
    QMC5883P_OSR_1
} qmc5883p_osr_t;

typedef enum {
    QMC5883P_DSR_1 = 0,         /* down-sample ratio (OSR2) */
    QMC5883P_DSR_2,
    QMC5883P_DSR_4,
    QMC5883P_DSR_8
} qmc5883p_dsr_t;

typedef enum {
    QMC5883P_MODE_SUSPEND    = 0,
    QMC5883P_MODE_CONTINUOUS = 3
} qmc5883p_mode_t;

typedef struct {
    qmc5883p_range_t range;
    qmc5883p_odr_t   odr;
    qmc5883p_osr_t   osr;
    qmc5883p_dsr_t   dsr;
    qmc5883p_mode_t  mode;
} qmc5883p_config_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883p_raw_t;

typedef struct {
    float x;            /* gauss, calibrated */
    float y;
    float z;
    float magnitude_g;  /* |B|; Earth's field is roughly 0.25 - 0.65 G */
    float heading_deg;  /* 0..360, atan2(y, x) + declination */
} qmc5883p_data_t;

typedef struct {
    i2c_t           *bus;
    uint8_t          addr;
    qmc5883p_range_t range;
    qmc5883p_odr_t   odr;
    qmc5883p_osr_t   osr;
    qmc5883p_dsr_t   dsr;
    qmc5883p_mode_t  mode;
    float            lsb_per_gauss;
    float            offset[3];     /* raw counts (hard iron) */
    float            scale[3];      /* per-axis (soft iron) */
    float            declination_deg;
    i2c_status_t     last_i2c;
    bool             initialized;
} qmc5883p_t;

/* Override for an RTOS / timer based delay */
void qmc5883p_delay_ms(uint32_t ms);

void qmc5883p_get_default_config(qmc5883p_config_t *cfg);

qmc5883p_status_t qmc5883p_read_id(qmc5883p_t *dev, uint8_t *id);
qmc5883p_status_t qmc5883p_init(qmc5883p_t *dev, i2c_t *bus, const qmc5883p_config_t *cfg);

qmc5883p_status_t qmc5883p_set_range(qmc5883p_t *dev, qmc5883p_range_t range);
qmc5883p_status_t qmc5883p_set_odr(qmc5883p_t *dev, qmc5883p_odr_t odr);
qmc5883p_status_t qmc5883p_set_mode(qmc5883p_t *dev, qmc5883p_mode_t mode);

qmc5883p_status_t qmc5883p_data_ready(qmc5883p_t *dev, bool *ready);
qmc5883p_status_t qmc5883p_read_raw(qmc5883p_t *dev, qmc5883p_raw_t *raw);
qmc5883p_status_t qmc5883p_read(qmc5883p_t *dev, qmc5883p_data_t *data);

/* Min/max calibration: rotate sensor through all orientations while this runs */
qmc5883p_status_t qmc5883p_calibrate(qmc5883p_t *dev, uint16_t samples);
qmc5883p_status_t qmc5883p_set_calibration(qmc5883p_t *dev, const float offset[3], const float scale[3]);
qmc5883p_status_t qmc5883p_get_calibration(const qmc5883p_t *dev, float offset[3], float scale[3]);
qmc5883p_status_t qmc5883p_set_declination(qmc5883p_t *dev, float deg);

#ifdef __cplusplus
}
#endif

#endif