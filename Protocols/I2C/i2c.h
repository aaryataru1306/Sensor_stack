#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    I2C_OK = 0,
    I2C_ERR_ARG,
    I2C_ERR_BUSY,
    I2C_ERR_TIMEOUT,
    I2C_ERR_NACK,
    I2C_ERR_BUS
} i2c_status_t;

typedef struct {
    I2C_TypeDef  *instance;
    GPIO_TypeDef *scl_port;
    uint8_t       scl_pin;
    GPIO_TypeDef *sda_port;
    uint8_t       sda_pin;
    uint8_t       af;
    bool          internal_pullup;
    uint32_t      pclk1_hz;
    uint32_t      speed_hz;
    uint32_t      timeout;
} i2c_config_t;

typedef struct {
    i2c_config_t cfg;
    uint32_t     timeout;
    uint32_t     cr2;
    uint32_t     ccr;
    uint32_t     trise;
    bool         initialized;
} i2c_t;

i2c_status_t i2c_init(i2c_t *dev, const i2c_config_t *cfg);
void         i2c_deinit(i2c_t *dev);
i2c_status_t i2c_recover(i2c_t *dev);

i2c_status_t i2c_probe(i2c_t *dev, uint8_t addr);
uint8_t      i2c_scan(i2c_t *dev, uint8_t *found, uint8_t max_found);

i2c_status_t i2c_write(i2c_t *dev, uint8_t addr, const uint8_t *data, uint16_t len);
i2c_status_t i2c_read(i2c_t *dev, uint8_t addr, uint8_t *data, uint16_t len);
i2c_status_t i2c_write_read(i2c_t *dev, uint8_t addr, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len);

i2c_status_t i2c_mem_write(i2c_t *dev, uint8_t addr, uint16_t reg, uint8_t reg_size, const uint8_t *data, uint16_t len);
i2c_status_t i2c_mem_read(i2c_t *dev, uint8_t addr, uint16_t reg, uint8_t reg_size, uint8_t *data, uint16_t len);

i2c_status_t i2c_write_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t value);
i2c_status_t i2c_read_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t *value);
i2c_status_t i2c_modify_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t mask, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif