#ifndef BMP388_CONFIG_H
#define BMP388_CONFIG_H

/*============================================================================*
 *                            HARDWARE PIN MAPPING                            *
 *============================================================================*/

/**
 * @brief Chip Select Pin for BMP388 on SPI1.
 * @note  On the STM32F411 Black Pill, PA4 is typically used for SPI1 CS.
 *        Adjust this number if your SPI driver expects a pin number (e.g., 4) 
 *        or a bitmask (e.g., (1U << 4)).
 */
#define BMP388_CS_PIN               4U

/*============================================================================*
 *                        DEFAULT SENSOR CONFIGURATION                        *
 *============================================================================*/

/**
 * @brief Pressure oversampling setting.
 *        0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x, 4 = 16x, 5 = 32x.
 */
#define BMP388_DEFAULT_OSR_PRESS    2U  /* 4x Oversampling */

/**
 * @brief Temperature oversampling setting.
 *        0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x, 4 = 16x, 5 = 32x.
 */
#define BMP388_DEFAULT_OSR_TEMP     1U  /* 2x Oversampling */

/**
 * @brief Output Data Rate (ODR).
 *        0x00 = 200 Hz, 0x01 = 100 Hz, 0x02 = 50 Hz, 0x03 = 25 Hz.
 */
#define BMP388_DEFAULT_ODR          0x02U /* 50 Hz */

#endif /* BMP388_CONFIG_H */