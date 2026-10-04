#ifndef BMP388_PRIVATE_H
#define BMP388_PRIVATE_H

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

/* Default Values & Commands */
#define BMP388_CHIP_ID_VALUE    0x50
#define BMP388_SOFT_RESET_CMD   0xB6

/* ========== Power Control Bitmasks (Register 0x1B) ========== */
#define BMP388_PWR_CTRL_PRESS_EN    (1U << 0)
#define BMP388_PWR_CTRL_TEMP_EN     (1U << 1)
#define BMP388_PWR_CTRL_MODE_NORMAL (0x03U << 4)

/* ========== OSR Bitfield Helper (Register 0x1C) ========== */
#define BMP388_OSR_BYTE(osr_p, osr_t) ((uint8_t)(((osr_t) & 0x07U) << 3 | ((osr_p) & 0x07U)))

/* ========== Calibration Divisors & Biases (BST-BMP388-DS001 §9.3.1) ========== */
#define BMP388_CALIB_T1_DIV     0.00390625f
#define BMP388_CALIB_T2_DIV     1073741824.0f
#define BMP388_CALIB_T3_DIV     281474976710656.0f

#define BMP388_CALIB_P1_BIAS    16384.0f
#define BMP388_CALIB_P1_DIV     1048576.0f
#define BMP388_CALIB_P2_BIAS    16384.0f
#define BMP388_CALIB_P2_DIV     536870912.0f
#define BMP388_CALIB_P3_DIV     4294967296.0f
#define BMP388_CALIB_P4_DIV     137438953472.0f
#define BMP388_CALIB_P5_DIV     0.125f
#define BMP388_CALIB_P6_DIV     64.0f
#define BMP388_CALIB_P7_DIV     256.0f
#define BMP388_CALIB_P8_DIV     32768.0f
#define BMP388_CALIB_P9_DIV     281474976710656.0f
#define BMP388_CALIB_P10_DIV    281474976710656.0f
#define BMP388_CALIB_P11_DIV    36893488147419103232.0f

#endif /*BMP388_PRIVATE_H*/