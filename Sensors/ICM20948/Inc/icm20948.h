/*
 * icm20948.h
 *
 * Driver for TDK InvenSense ICM-20948 (9-axis IMU) over SPI
 * Built on top of the user's own register-level SPI1 driver (spi_driver.h/.c)
 * Target: STM32F411CEU6
 *
 * Place this file next to spi_driver.h (e.g. Core/Inc)
 */

#ifndef ICM20948_H_
#define ICM20948_H_

#include "spi_driver.h"
#include <stdint.h>

/* ============ REGISTER BANK SELECT (common to all banks) ============ */
#define ICM20948_REG_BANK_SEL    0x7F

/* ============ BANK 0 REGISTERS ============ */
#define ICM20948_B0_WHO_AM_I         0x00
#define ICM20948_B0_USER_CTRL        0x03
#define ICM20948_B0_PWR_MGMT_1       0x06
#define ICM20948_B0_PWR_MGMT_2       0x07
#define ICM20948_B0_ACCEL_XOUT_H     0x2D
#define ICM20948_B0_GYRO_XOUT_H      0x33
#define ICM20948_B0_TEMP_OUT_H       0x39

#define ICM20948_WHO_AM_I_VALUE      0xEA   /* expected WHO_AM_I value */

/* ============ BANK 2 REGISTERS ============ */
#define ICM20948_B2_GYRO_SMPLRT_DIV      0x00
#define ICM20948_B2_GYRO_CONFIG_1        0x01
#define ICM20948_B2_ACCEL_SMPLRT_DIV_1   0x10
#define ICM20948_B2_ACCEL_SMPLRT_DIV_2   0x11
#define ICM20948_B2_ACCEL_CONFIG         0x14

/* ============ ENUMS FOR RANGES ============ */
typedef enum {
    ICM20948_GYRO_FS_250DPS  = 0x00,
    ICM20948_GYRO_FS_500DPS  = 0x01,
    ICM20948_GYRO_FS_1000DPS = 0x02,
    ICM20948_GYRO_FS_2000DPS = 0x03
} ICM20948_GyroFS_t;

typedef enum {
    ICM20948_ACCEL_FS_2G  = 0x00,
    ICM20948_ACCEL_FS_4G  = 0x01,
    ICM20948_ACCEL_FS_8G  = 0x02,
    ICM20948_ACCEL_FS_16G = 0x03
} ICM20948_AccelFS_t;

typedef enum {
    ICM20948_OK = 0,
    ICM20948_ERROR_WHOAMI = 1
} ICM20948_Status_t;

/* ============ DATA STRUCTURE ============ */
typedef struct {
    float accel_x, accel_y, accel_z;   /* in g */
    float gyro_x, gyro_y, gyro_z;      /* in dps */
    float temp_c;                       /* in Celsius */
} ICM20948_Data_t;

/* ============ PUBLIC FUNCTIONS ============ */
ICM20948_Status_t ICM20948_Init(ICM20948_AccelFS_t accel_fs, ICM20948_GyroFS_t gyro_fs);
uint8_t ICM20948_WhoAmI(void);
void ICM20948_ReadAll(ICM20948_Data_t *data);
void ICM20948_ReadAccel(float *ax, float *ay, float *az);
void ICM20948_ReadGyro(float *gx, float *gy, float *gz);
void ICM20948_ReadTemp(float *temp_c);

#endif /* ICM20948_H_ */
