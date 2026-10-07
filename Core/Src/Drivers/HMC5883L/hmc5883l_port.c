#include "ErrTypes.h"
#include "I2C_interface.h"
#include "hmc5883l.h"

#define HMC_MAX_WRITE 8

static int port_write(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t buf[1 + HMC_MAX_WRITE];
    if (len > HMC_MAX_WRITE) return -1;

    buf[0] = reg;
    for (uint16_t i = 0; i < len; i++) buf[1 + i] = data[i];

    return (I2C_enumMasterTransmit((I2C_Config_t *)ctx, addr, buf, len + 1) == OK) ? 0 : -1;
}

static int port_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    return (I2C_enumMasterTransmitReceive((I2C_Config_t *)ctx, addr, &reg, 1, data, len) == OK) ? 0 : -1;
}

const hmc_bus_t hmc_i2c_bus_template = { 0, port_write, port_read };
