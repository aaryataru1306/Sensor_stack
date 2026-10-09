#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_AD0_LOW   0x68U
#define MPU6050_ADDR_AD0_HIGH  0x69U
#define MPU6050_WHO_AM_I_VALUE 0x68U

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_ARG,
    MPU6050_ERR_BUS,
    MPU6050_ERR_ID
} mpu6050_status_t;

typedef enum {
    MPU6050_ACCEL_2G = 0,
    MPU6050_ACCEL_4G,
    MPU6050_ACCEL_8G,
    MPU6050_ACCEL_16G
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_250DPS = 0,
    MPU6050_GYRO_500DPS,
    MPU6050_GYRO_1000DPS,
    MPU6050_GYRO_2000DPS
} mpu6050_gyro_range_t;

typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ,
    MPU6050_DLPF_94HZ,
    MPU6050_DLPF_44HZ,
    MPU6050_DLPF_21HZ,
    MPU6050_DLPF_10HZ,
    MPU6050_DLPF_5HZ
} mpu6050_dlpf_t;

typedef struct {
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t  gyro_range;
    mpu6050_dlpf_t        dlpf;
    uint16_t              sample_rate_hz;
} mpu6050_config_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw_t;

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temp_c;
} mpu6050_data_t;

typedef struct {
    i2c_t                *bus;
    uint8_t               addr;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t  gyro_range;
    mpu6050_dlpf_t        dlpf;
    uint16_t              sample_rate_hz;
    uint32_t              sample_period_ms;
    float                 accel_lsb_per_g;
    float                 gyro_lsb_per_dps;
    int16_t               gyro_offset[3];
    i2c_status_t          last_i2c;
    bool                  initialized;
} mpu6050_t;

void mpu6050_delay_ms(uint32_t ms);

void             mpu6050_get_default_config(mpu6050_config_t *cfg);
mpu6050_status_t mpu6050_init(mpu6050_t *dev, i2c_t *bus, uint8_t addr, const mpu6050_config_t *cfg);
mpu6050_status_t mpu6050_who_am_i(mpu6050_t *dev, uint8_t *id);

mpu6050_status_t mpu6050_set_accel_range(mpu6050_t *dev, mpu6050_accel_range_t range);
mpu6050_status_t mpu6050_set_gyro_range(mpu6050_t *dev, mpu6050_gyro_range_t range);
mpu6050_status_t mpu6050_set_dlpf(mpu6050_t *dev, mpu6050_dlpf_t dlpf);
mpu6050_status_t mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t hz);

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);
mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *data);

mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t samples);
mpu6050_status_t mpu6050_set_gyro_offset(mpu6050_t *dev, int16_t x, int16_t y, int16_t z);
mpu6050_status_t mpu6050_get_gyro_offset(const mpu6050_t *dev, int16_t offset[3]);

mpu6050_status_t mpu6050_sleep(mpu6050_t *dev, bool enable);
mpu6050_status_t mpu6050_enable_data_ready_int(mpu6050_t *dev, bool enable);
mpu6050_status_t mpu6050_data_ready(mpu6050_t *dev, bool *ready);

#ifdef __cplusplus
}
#endif

#endif