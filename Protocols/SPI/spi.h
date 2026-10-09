#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "stm32f411xe.h"


#ifdef __cplusplus
extern "C" {
#endif

/* SPI1 Pin Mapping (Fixed to GPIOA) */
#define SPI1_SCK_PIN          5U   /* PA5 - SCK */
#define SPI1_MISO_PIN         6U   /* PA6 - MISO */
#define SPI1_MOSI_PIN         7U   /* PA7 - MOSI */

#define SPI1_CS_ICM20948_PIN  1U   /* PA1 - ICM20948 CS */
#define SPI1_CS_BMP388_PIN    4U   /* PA4 - BMP388 CS */
#define SPI1_CS_Enable(pin)  SPI_CS_Select(pin)
#define SPI1_CS_Disable(pin) SPI_CS_Deselect(pin)

/* --- Initialization & CS Helpers --- */
void SPI1_Init(void);
void SPI_CS_Init(uint8_t pinNumber);
void SPI_CS_Select(uint8_t pinNumber);
void SPI_CS_Deselect(uint8_t pinNumber);

/* --- Core Bus Transfer APIs --- */
uint8_t SPI1_TransferByte(uint8_t data);
void SPI1_ReadRegisters(uint8_t cs_pin, uint8_t reg_addr, uint8_t *buffer, uint16_t length);
void SPI1_WriteRegister(uint8_t cs_pin, uint8_t reg_addr, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif