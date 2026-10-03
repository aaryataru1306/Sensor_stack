#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>

/* ========== Registers ========== */
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_ERR_REG      0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA_0       0x04
#define BMP388_REG_EVENT        0x10
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CONFIG       0x1F
#define BMP388_REG_CALIB_DATA   0x31
#define BMP388_REG_CMD          0x7E

#define BMP388_CHIP_ID_VALUE    0x50
#define BMP388_SOFT_RESET_CMD   0xB6

/* ========== Structs ========== */
typedef struct {
    float par_t1;
    float par_t2;
    float par_t3;
    float par_p1;
    float par_p2;
    float par_p3;
    float par_p4;
    float par_p5;
    float par_p6;
    float par_p7;
    float par_p8;
    float par_p9;
    float par_p10;
    float par_p11;
    float t_lin;
} BMP388_CalibData;

typedef struct {
    float pressure;     /* Pa */
    float temperature;  /* Celsius */
} BMP388_Data;

/* ========== API ========== */
uint8_t BMP388_Init(BMP388_CalibData *calib);
uint8_t BMP388_ReadChipID(void);
void    BMP388_SoftReset(void);
void    BMP388_ReadCalibData(BMP388_CalibData *calib);
void    BMP388_SetPowerCtrl(uint8_t value);
void    BMP388_SetOSR(uint8_t osr_p, uint8_t osr_t);
void    BMP388_SetODR(uint8_t odr);
void    BMP388_ReadRawData(uint32_t *raw_press, uint32_t *raw_temp);
void    BMP388_GetData(BMP388_CalibData *calib, BMP388_Data *out);

#endif
