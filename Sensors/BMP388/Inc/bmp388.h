#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <stddef.h>
#include "i2c.h"

/* BMP388 I2C addresses (SDO connected to GND or VDDIO) */
#define BMP388_I2C_ADDR_PRIM    0x76U
#define BMP388_I2C_ADDR_SEC     0x77U

/* BMP388 register addresses */
#define BMP388_REG_CHIP_ID      0x00U
#define BMP388_REG_ERR_REG      0x02U
#define BMP388_REG_STATUS       0x03U
#define BMP388_REG_DATA_0       0x04U
#define BMP388_REG_PWR_CTRL     0x1BU
#define BMP388_REG_OSR          0x1CU
#define BMP388_REG_ODR          0x1DU
#define BMP388_REG_CONFIG       0x1FU
#define BMP388_REG_CMD          0x7EU

/* Expected sensor identification */
#define BMP388_CHIP_ID          0x50U

/* Driver return codes */
#define BMP388_OK               0
#define BMP388_ERROR            (-1)
#define BMP388_INVALID_ID       (-2)

/* Sensor data structure */
typedef struct
{
    float temperature_c;
    float pressure_pa;
    float altitude_m;
} BMP388_Data;

/* Public driver functions */
int BMP388_Init(i2c_t *i2c_dev, uint8_t i2c_addr);
int BMP388_ReadChipID(uint8_t *chip_id);
int BMP388_ReadData(BMP388_Data *data);

#endif /* BMP388_H */