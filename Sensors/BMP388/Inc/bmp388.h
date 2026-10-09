#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>

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

/* Sensor data */
typedef struct
{
    float temperature_c;
    float pressure_pa;
} BMP388_Data;

/* Public driver functions */
int BMP388_Init(void);
int BMP388_ReadChipID(uint8_t *chip_id);
int BMP388_ReadData(BMP388_Data *data);

#endif /* BMP388_H */