#include "i2c.h"
#include <stddef.h>

#define I2C_DRV_DEFAULT_TIMEOUT  50000U
#define I2C_DRV_RECOVER_CLOCKS   9U
#define I2C_DRV_ADDR_MAX         0x7FU
#define I2C_DRV_SCAN_FIRST       0x08U
#define I2C_DRV_SCAN_LAST        0x77U
#define I2C_DRV_MODE_OUT         1U
#define I2C_DRV_MODE_AF          2U
#define I2C_DRV_SR1_ERRORS       (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR)

static void delay_us(uint32_t us)
{
    volatile uint32_t n = ((SystemCoreClock / 4000000U) + 1U) * us;
    while (n--) {
        __NOP();
    }
}

static uint32_t instance_clock_mask(const I2C_TypeDef *r)
{
    if (r == I2C1) {
        return RCC_APB1ENR_I2C1EN;
    }
    if (r == I2C2) {
        return RCC_APB1ENR_I2C2EN;
    }
#ifdef I2C3
    if (r == I2C3) {
        return RCC_APB1ENR_I2C3EN;
    }
#endif
    return 0U;
}

static uint32_t gpio_clock_mask(const GPIO_TypeDef *p)
{
    return 1U << (((uint32_t)(uintptr_t)p - (uint32_t)GPIOA_BASE) >> 10);
}

static void gpio_config(GPIO_TypeDef *p, uint8_t pin, uint32_t mode, uint8_t af, bool pullup)
{
    uint32_t s2 = (uint32_t)pin * 2U;
    uint32_t s4 = (uint32_t)(pin & 7U) * 4U;

    p->OTYPER |= (1U << pin);
    p->OSPEEDR |= (3U << s2);
    p->PUPDR = (p->PUPDR & ~(3U << s2)) | ((pullup ? 1U : 0U) << s2);
    if (mode == I2C_DRV_MODE_AF) {
        p->AFR[pin >> 3U] = (p->AFR[pin >> 3U] & ~(0xFU << s4)) | ((uint32_t)af << s4);
    }
    p->MODER = (p->MODER & ~(3U << s2)) | (mode << s2);
}

static void pins_to_af(const i2c_t *d)
{
    gpio_config(d->cfg.scl_port, d->cfg.scl_pin, I2C_DRV_MODE_AF, d->cfg.af, d->cfg.internal_pullup);
    gpio_config(d->cfg.sda_port, d->cfg.sda_pin, I2C_DRV_MODE_AF, d->cfg.af, d->cfg.internal_pullup);
}

static i2c_status_t compute_timing(i2c_t *d)
{
    uint32_t pclk = d->cfg.pclk1_hz;
    uint32_t speed = d->cfg.speed_hz;
    uint32_t mhz = pclk / 1000000U;
    uint32_t ccr;

    if (mhz < 2U || mhz > 50U || speed == 0U || speed > 400000U) {
        return I2C_ERR_ARG;
    }

    if (speed > 100000U) {
        if (mhz < 4U) {
            return I2C_ERR_ARG;
        }
        ccr = (pclk + (3U * speed) - 1U) / (3U * speed);
        d->trise = ((mhz * 3U) / 10U) + 1U;
        d->ccr = ccr | I2C_CCR_FS;
    } else {
        ccr = (pclk + (2U * speed) - 1U) / (2U * speed);
        d->trise = mhz + 1U;
        d->ccr = ccr;
    }

    if (ccr < 4U || ccr > 0xFFFU) {
        return I2C_ERR_ARG;
    }

    d->cr2 = mhz;
    return I2C_OK;
}

static void periph_setup(const i2c_t *d)
{
    I2C_TypeDef *r = d->cfg.instance;

    r->CR1 &= ~I2C_CR1_PE;
    r->CR1 |= I2C_CR1_SWRST;
    r->CR1 &= ~I2C_CR1_SWRST;
    r->CR2 = d->cr2;
    r->CCR = d->ccr;
    r->TRISE = d->trise;
    r->OAR1 = (1U << 14);
    r->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
}

static bool wait_busy_clear(const i2c_t *d)
{
    uint32_t n = d->timeout;

    while (n--) {
        if ((d->cfg.instance->SR2 & I2C_SR2_BUSY) == 0U) {
            return true;
        }
    }
    return false;
}

static i2c_status_t wait_sr1(const i2c_t *d, uint32_t mask)
{
    I2C_TypeDef *r = d->cfg.instance;
    uint32_t n = d->timeout;
    uint32_t sr1;

    while (n--) {
        sr1 = r->SR1;
        if (sr1 & mask) {
            return I2C_OK;
        }
        if (sr1 & I2C_SR1_AF) {
            return I2C_ERR_NACK;
        }
        if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO)) {
            return I2C_ERR_BUS;
        }
    }
    return I2C_ERR_TIMEOUT;
}

static i2c_status_t fail(i2c_t *d, i2c_status_t st)
{
    I2C_TypeDef *r = d->cfg.instance;

    if (r->SR2 & I2C_SR2_MSL) {
        r->CR1 |= I2C_CR1_STOP;
    }
    r->SR1 &= ~I2C_DRV_SR1_ERRORS;
    if (!wait_busy_clear(d)) {
        (void)i2c_recover(d);
    }
    return st;
}

static bool dev_valid(const i2c_t *d, uint8_t addr)
{
    return (d != NULL) && d->initialized && (addr <= I2C_DRV_ADDR_MAX);
}

static i2c_status_t begin(i2c_t *d)
{
    I2C_TypeDef *r = d->cfg.instance;

    if (!wait_busy_clear(d)) {
        if (i2c_recover(d) != I2C_OK) {
            return I2C_ERR_BUSY;
        }
    }
    r->SR1 &= ~I2C_DRV_SR1_ERRORS;
    r->CR1 &= ~I2C_CR1_POS;
    r->CR1 |= I2C_CR1_ACK;
    return I2C_OK;
}

static i2c_status_t start_cond(const i2c_t *d)
{
    d->cfg.instance->CR1 |= I2C_CR1_START;
    return wait_sr1(d, I2C_SR1_SB);
}

static i2c_status_t send_addr(const i2c_t *d, uint8_t addr, uint8_t rw)
{
    d->cfg.instance->DR = (uint32_t)((uint32_t)addr << 1) | rw;
    return wait_sr1(d, I2C_SR1_ADDR);
}

static void clear_addr(const I2C_TypeDef *r)
{
    (void)r->SR1;
    (void)r->SR2;
}

static i2c_status_t tx_bytes(const i2c_t *d, const uint8_t *p, uint16_t n)
{
    i2c_status_t st;

    while (n--) {
        st = wait_sr1(d, I2C_SR1_TXE);
        if (st != I2C_OK) {
            return st;
        }
        d->cfg.instance->DR = *p++;
    }
    return I2C_OK;
}

static i2c_status_t write_phase(i2c_t *d, uint8_t addr, const uint8_t *a, uint16_t a_len, const uint8_t *b, uint16_t b_len, bool stop)
{
    I2C_TypeDef *r = d->cfg.instance;
    i2c_status_t st;

    st = start_cond(d);
    if (st != I2C_OK) {
        return st;
    }
    st = send_addr(d, addr, 0U);
    if (st != I2C_OK) {
        return st;
    }
    clear_addr(r);

    st = tx_bytes(d, a, a_len);
    if (st != I2C_OK) {
        return st;
    }
    st = tx_bytes(d, b, b_len);
    if (st != I2C_OK) {
        return st;
    }

    if ((a_len + b_len) != 0U) {
        st = wait_sr1(d, I2C_SR1_BTF);
        if (st != I2C_OK) {
            return st;
        }
    }
    if (stop) {
        r->CR1 |= I2C_CR1_STOP;
    }
    return I2C_OK;
}

static i2c_status_t read_phase(i2c_t *d, uint8_t addr, uint8_t *buf, uint16_t len)
{
    I2C_TypeDef *r = d->cfg.instance;
    i2c_status_t st;
    uint32_t primask;

    r->CR1 |= I2C_CR1_ACK;
    r->CR1 &= ~I2C_CR1_POS;

    st = start_cond(d);
    if (st != I2C_OK) {
        return st;
    }
    st = send_addr(d, addr, 1U);
    if (st != I2C_OK) {
        return st;
    }

    if (len == 1U) {
        r->CR1 &= ~I2C_CR1_ACK;
        primask = __get_PRIMASK();
        __disable_irq();
        clear_addr(r);
        r->CR1 |= I2C_CR1_STOP;
        __set_PRIMASK(primask);
        st = wait_sr1(d, I2C_SR1_RXNE);
        if (st != I2C_OK) {
            return st;
        }
        buf[0] = (uint8_t)r->DR;
        return I2C_OK;
    }

    if (len == 2U) {
        r->CR1 &= ~I2C_CR1_ACK;
        r->CR1 |= I2C_CR1_POS;
        clear_addr(r);
        st = wait_sr1(d, I2C_SR1_BTF);
        if (st != I2C_OK) {
            return st;
        }
        primask = __get_PRIMASK();
        __disable_irq();
        r->CR1 |= I2C_CR1_STOP;
        buf[0] = (uint8_t)r->DR;
        __set_PRIMASK(primask);
        buf[1] = (uint8_t)r->DR;
        r->CR1 &= ~I2C_CR1_POS;
        return I2C_OK;
    }

    clear_addr(r);

    while (len > 3U) {
        st = wait_sr1(d, I2C_SR1_RXNE);
        if (st != I2C_OK) {
            return st;
        }
        *buf++ = (uint8_t)r->DR;
        len--;
    }

    st = wait_sr1(d, I2C_SR1_BTF);
    if (st != I2C_OK) {
        return st;
    }
    r->CR1 &= ~I2C_CR1_ACK;
    primask = __get_PRIMASK();
    __disable_irq();
    buf[0] = (uint8_t)r->DR;
    r->CR1 |= I2C_CR1_STOP;
    buf[1] = (uint8_t)r->DR;
    __set_PRIMASK(primask);
    st = wait_sr1(d, I2C_SR1_RXNE);
    if (st != I2C_OK) {
        return st;
    }
    buf[2] = (uint8_t)r->DR;
    return I2C_OK;
}

static uint8_t build_reg(uint16_t reg, uint8_t reg_size, uint8_t *out)
{
    if (reg_size == 2U) {
        out[0] = (uint8_t)(reg >> 8);
        out[1] = (uint8_t)(reg & 0xFFU);
        return 2U;
    }
    out[0] = (uint8_t)(reg & 0xFFU);
    return 1U;
}

i2c_status_t i2c_init(i2c_t *dev, const i2c_config_t *cfg)
{
    uint32_t mask;
    i2c_status_t st;

    if (dev == NULL || cfg == NULL || cfg->instance == NULL ||
        cfg->scl_port == NULL || cfg->sda_port == NULL ||
        cfg->scl_pin > 15U || cfg->sda_pin > 15U || cfg->af > 15U) {
        return I2C_ERR_ARG;
    }

    mask = instance_clock_mask(cfg->instance);
    if (mask == 0U) {
        return I2C_ERR_ARG;
    }

    dev->initialized = false;
    dev->cfg = *cfg;
    dev->timeout = (cfg->timeout != 0U) ? cfg->timeout : I2C_DRV_DEFAULT_TIMEOUT;

    st = compute_timing(dev);
    if (st != I2C_OK) {
        return st;
    }

    RCC->AHB1ENR |= gpio_clock_mask(cfg->scl_port) | gpio_clock_mask(cfg->sda_port);
    RCC->APB1ENR |= mask;
    (void)RCC->APB1ENR;

    pins_to_af(dev);
    periph_setup(dev);
    dev->initialized = true;

    if (!wait_busy_clear(dev)) {
        return i2c_recover(dev);
    }
    return I2C_OK;
}

void i2c_deinit(i2c_t *dev)
{
    if (dev == NULL || !dev->initialized) {
        return;
    }
    dev->cfg.instance->CR1 &= ~I2C_CR1_PE;
    RCC->APB1ENR &= ~instance_clock_mask(dev->cfg.instance);
    dev->initialized = false;
}

i2c_status_t i2c_recover(i2c_t *dev)
{
    GPIO_TypeDef *sp;
    GPIO_TypeDef *dp;
    uint32_t scl;
    uint32_t sda;
    uint32_t i;

    if (dev == NULL || dev->cfg.instance == NULL) {
        return I2C_ERR_ARG;
    }

    sp = dev->cfg.scl_port;
    dp = dev->cfg.sda_port;
    scl = 1U << dev->cfg.scl_pin;
    sda = 1U << dev->cfg.sda_pin;

    dev->cfg.instance->CR1 &= ~I2C_CR1_PE;

    sp->BSRR = scl;
    dp->BSRR = sda;
    gpio_config(sp, dev->cfg.scl_pin, I2C_DRV_MODE_OUT, 0U, dev->cfg.internal_pullup);
    gpio_config(dp, dev->cfg.sda_pin, I2C_DRV_MODE_OUT, 0U, dev->cfg.internal_pullup);
    delay_us(5U);

    for (i = 0U; i < I2C_DRV_RECOVER_CLOCKS; i++) {
        if (dp->IDR & sda) {
            break;
        }
        sp->BSRR = scl << 16;
        delay_us(5U);
        sp->BSRR = scl;
        delay_us(5U);
    }

    sp->BSRR = scl << 16;
    delay_us(5U);
    dp->BSRR = sda << 16;
    delay_us(5U);
    sp->BSRR = scl;
    delay_us(5U);
    dp->BSRR = sda;
    delay_us(5U);

    pins_to_af(dev);
    periph_setup(dev);
    delay_us(10U);

    return (dev->cfg.instance->SR2 & I2C_SR2_BUSY) ? I2C_ERR_BUSY : I2C_OK;
}

i2c_status_t i2c_write(i2c_t *dev, uint8_t addr, const uint8_t *data, uint16_t len)
{
    i2c_status_t st;

    if (!dev_valid(dev, addr) || (len != 0U && data == NULL)) {
        return I2C_ERR_ARG;
    }
    st = begin(dev);
    if (st != I2C_OK) {
        return st;
    }
    st = write_phase(dev, addr, data, len, NULL, 0U, true);
    return (st == I2C_OK) ? I2C_OK : fail(dev, st);
}

i2c_status_t i2c_read(i2c_t *dev, uint8_t addr, uint8_t *data, uint16_t len)
{
    i2c_status_t st;

    if (!dev_valid(dev, addr) || data == NULL || len == 0U) {
        return I2C_ERR_ARG;
    }
    st = begin(dev);
    if (st != I2C_OK) {
        return st;
    }
    st = read_phase(dev, addr, data, len);
    return (st == I2C_OK) ? I2C_OK : fail(dev, st);
}

i2c_status_t i2c_write_read(i2c_t *dev, uint8_t addr, const uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t rx_len)
{
    i2c_status_t st;

    if (!dev_valid(dev, addr) || rx == NULL || rx_len == 0U || (tx_len != 0U && tx == NULL)) {
        return I2C_ERR_ARG;
    }
    st = begin(dev);
    if (st != I2C_OK) {
        return st;
    }
    if (tx_len != 0U) {
        st = write_phase(dev, addr, tx, tx_len, NULL, 0U, false);
        if (st != I2C_OK) {
            return fail(dev, st);
        }
    }
    st = read_phase(dev, addr, rx, rx_len);
    return (st == I2C_OK) ? I2C_OK : fail(dev, st);
}

i2c_status_t i2c_mem_write(i2c_t *dev, uint8_t addr, uint16_t reg, uint8_t reg_size, const uint8_t *data, uint16_t len)
{
    uint8_t hdr[2];
    uint8_t hlen;
    i2c_status_t st;

    if (!dev_valid(dev, addr) || (reg_size != 1U && reg_size != 2U) || (len != 0U && data == NULL)) {
        return I2C_ERR_ARG;
    }
    hlen = build_reg(reg, reg_size, hdr);
    st = begin(dev);
    if (st != I2C_OK) {
        return st;
    }
    st = write_phase(dev, addr, hdr, hlen, data, len, true);
    return (st == I2C_OK) ? I2C_OK : fail(dev, st);
}

i2c_status_t i2c_mem_read(i2c_t *dev, uint8_t addr, uint16_t reg, uint8_t reg_size, uint8_t *data, uint16_t len)
{
    uint8_t hdr[2];
    uint8_t hlen;

    if (reg_size != 1U && reg_size != 2U) {
        return I2C_ERR_ARG;
    }
    hlen = build_reg(reg, reg_size, hdr);
    return i2c_write_read(dev, addr, hdr, hlen, data, len);
}

i2c_status_t i2c_write_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;
    return i2c_write(dev, addr, buf, 2U);
}

i2c_status_t i2c_read_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t *value)
{
    return i2c_write_read(dev, addr, &reg, 1U, value, 1U);
}

i2c_status_t i2c_modify_reg(i2c_t *dev, uint8_t addr, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t v;
    i2c_status_t st;

    st = i2c_read_reg(dev, addr, reg, &v);
    if (st != I2C_OK) {
        return st;
    }
    v = (uint8_t)((v & (uint8_t)~mask) | (value & mask));
    return i2c_write_reg(dev, addr, reg, v);
}

i2c_status_t i2c_probe(i2c_t *dev, uint8_t addr)
{
    return i2c_write(dev, addr, NULL, 0U);
}

uint8_t i2c_scan(i2c_t *dev, uint8_t *found, uint8_t max_found)
{
    uint8_t count = 0U;
    uint8_t a;

    if (dev == NULL || found == NULL) {
        return 0U;
    }
    for (a = I2C_DRV_SCAN_FIRST; a <= I2C_DRV_SCAN_LAST && count < max_found; a++) {
        if (i2c_probe(dev, a) == I2C_OK) {
            found[count++] = a;
        }
    }
    return count;
}