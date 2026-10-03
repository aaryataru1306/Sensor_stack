/**
 ******************************************************************************
 * @file    i2c_program.c
 * @brief   Implementation of the bare-metal I2C master protocol layer
 *          declared in i2c_interface.h.
 * @details
 *   Register-level driver for STM32F411 I2C1 / I2C2 / I2C3, written
 *   against RM0383 §22. No HAL_* calls, no higher-layer dependency.
 *
 *   Bus topology assumed on the Black Pill:
 *     I2C1  PB6 (SCL) / PB7 (SDA)   AF4   open-drain
 *     I2C2  PB10 (SCL) / PB3 (SDA)  AF4   open-drain
 *     I2C3  PA8 (SCL) / PB4 (SDA)   AF4   open-drain
 *
 *   Only I2C1's GPIO bring-up is actually exercised by this project
 *   (MPU6050 sits on PB6/PB7). I2C2 and I2C3 are here because the
 *   peripheral-level code path is identical — same register block, same
 *   timing model — and having all three avoids a "why does I2C2 return
 *   NULL_POINTER?" surprise when a second I2C sensor is added later.
 *
 *   Timing model: the F411's I2C peripherals use the classic CCR/TRISE
 *   scheme (NOT the newer TIMINGR single-register model from the F0/F3/
 *   F7/L4 families). CCR is computed from PCLK1 and the target SCL rate;
 *   TRISE is set from the maximum allowed rise time for the chosen
 *   speed mode. Both are covered in RM0383 §22.6.9–§22.6.10.
 *
 *   Error mapping: every I2C error flag that can plausibly show up
 *   during a master transfer (AF, BERR, ARLO, OVR) is checked on every
 *   wait, and any of them aborts the transaction with the appropriate
 *   ErrorState_t. AF — NACK from the slave — is the one that actually
 *   fires in practice (wrong address, part not populated, bus stuck);
 *   the others are here so a bus fault can never wedge the CPU in a
 *   wait loop.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#include "i2c_interface.h"
#include "i2c_private.h"

/*============================================================================*
 *                          RCC REGISTER BITS                                 *
 *============================================================================*/

/**
 * @details
 *   Base addresses here mirror the ones in spi_driver.c. Only the bits
 *   actually used by this driver are named — the RCC block has dozens
 *   more, but they belong to other peripherals.
 */
#define RCC_BASE             0x40023800UL
#define RCC_APB1ENR          (*(volatile uint32_t *)(RCC_BASE + 0x40UL))
#define RCC_AHB1ENR          (*(volatile uint32_t *)(RCC_BASE + 0x30UL))

#define RCC_APB1ENR_I2C1EN   (1UL << 21)
#define RCC_APB1ENR_I2C2EN   (1UL << 22)
#define RCC_APB1ENR_I2C3EN   (1UL << 23)

#define RCC_AHB1ENR_GPIOAEN  (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN  (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN  (1UL << 2)

/*============================================================================*
 *                          GPIO REGISTER LAYOUT                              *
 *============================================================================*/

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOA_BASE   0x40020000UL
#define GPIOB_BASE   0x40020400UL
#define GPIOC_BASE   0x40020800UL

#define GPIOA        ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB        ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC        ((GPIO_TypeDef *)GPIOC_BASE)

/* MODER field values */
#define GPIO_MODER_INPUT   0U
#define GPIO_MODER_OUTPUT  1U
#define GPIO_MODER_AF      2U
#define GPIO_MODER_ANALOG  3U

/* OTYPER */
#define GPIO_OTYPE_PP      0U
#define GPIO_OTYPE_OD      1U

/* OSPEEDR */
#define GPIO_OSPEED_LOW    0U
#define GPIO_OSPEED_MED    1U
#define GPIO_OSPEED_HIGH   2U
#define GPIO_OSPEED_VHIGH  3U

/* PUPDR */
#define GPIO_PUPDR_NONE    0U
#define GPIO_PUPDR_PU      1U
#define GPIO_PUPDR_PD      2U

/** @brief AF4 is the alternate-function number that selects I2C1/2/3 on
 *         every STM32F4 part that exposes them on GPIO. */
#define GPIO_AF_I2C        4U

/*============================================================================*
 *                          TIMEOUT BUDGET                                    *
 *============================================================================*/

/**
 * @brief Loop-iteration budget for every flag wait.
 *
 * @details
 *   A raw loop count, not milliseconds — the tight polling loop runs a
 *   roughly predictable number of iterations per microsecond at a given
 *   core clock, and this is comfortably above the worst-case phase time
 *   at 400 kHz SCL with a slow slave that stretches the clock.
 *
 *   If a wait ever exceeds this, the transaction is aborted with
 *   TIMEOUT_STATE and the bus is left in a known state (STOP issued on
 *   a best-effort basis) so the next transaction can retry cleanly.
 *   This is what stops a stuck bus from hanging the whole main loop.
 */
#define I2C_TIMEOUT_ITERS   100000UL

/*============================================================================*
 *                          CHANNEL → BASE POINTER                            *
 *============================================================================*/

static I2C_Regs_t *i2c_base(I2C_Channel_t ch)
{
    switch (ch) {
        case I2C_CHANNEL1: return (I2C_Regs_t *)I2C1_BASE_ADDR;
        case I2C_CHANNEL2: return (I2C_Regs_t *)I2C2_BASE_ADDR;
        case I2C_CHANNEL3: return (I2C_Regs_t *)I2C3_BASE_ADDR;
        default:           return 0;
    }
}

/*============================================================================*
 *                          LOW-LEVEL HELPERS                                 *
 *============================================================================*/

/**
 * @brief Wait until (reg & mask) != 0, or a bus error is detected.
 *
 * @retval OK             The bit became 1.
 * @retval NOK            A bus error (AF, BERR, ARLO) or OVR was detected.
 * @retval TIMEOUT_STATE  The bit never became 1 within the iteration budget.
 */
static ErrorState_t i2c_wait_flag_set(I2C_Regs_t *i2c, uint32_t mask)
{
    for (uint32_t i = 0U; i < I2C_TIMEOUT_ITERS; i++) {
        /* Check for hard errors first — they abort regardless of the
         * bit we're waiting for. */
        if (i2c->SR1 & I2C_SR1_AF)   return NOK;   /* NACK */
        if (i2c->SR1 & I2C_SR1_BERR) return NOK;   /* misplaced START/STOP */
        if (i2c->SR1 & I2C_SR1_ARLO) return NOK;   /* arbitration lost */

        if (i2c->SR1 & mask) {
            return OK;
        }
    }
    return TIMEOUT_STATE;
}

/**
 * @brief Wait until (reg & mask) == 0, or a bus error is detected.
 *
 * @details
 *   Used for the BUSY flag (SR2). Unlike SR1 bits, BUSY=0 means "bus
 *   is idle" — so the sense of the loop is inverted.
 */
static ErrorState_t i2c_wait_flag_clear(I2C_Regs_t *i2c, uint32_t mask)
{
    for (uint32_t i = 0U; i < I2C_TIMEOUT_ITERS; i++) {
        if (i2c->SR1 & I2C_SR1_AF)   return NOK;
        if (i2c->SR1 & I2C_SR1_BERR) return NOK;
        if (i2c->SR1 & I2C_SR1_ARLO) return NOK;

        if (!(i2c->SR2 & mask)) {
            return OK;
        }
    }
    return TIMEOUT_STATE;
}

/**
 * @brief Clear the sticky error flags (AF, BERR, ARLO, OVR).
 *
 * @details
 *   Per RM0383 §22.6.6, these are cleared by writing 0 to the bit —
 *   which is a "clear by writing zero" register, unlike most of the
 *   peripheral's registers. Reading SR1 followed by writing SR1 with
 *   the specific bits cleared is the standard pattern; the whole lower
 *   16 bits are cleared at once here, which is safe because the only
 *   set-able-and-not-auto-cleared bits in that range are exactly the
 *   error flags we want gone.
 */
static void i2c_clear_error_flags(I2C_Regs_t *i2c)
{
    (void)i2c->SR1;
    i2c->SR1 = 0x00000000U;
}

/**
 * @brief Read SR1 then SR2 — the two-register dance that clears ADDR.
 *
 * @details
 *   ADDR is a "read SR1, then read SR2" flag: neither read alone clears
 *   it, and clearing it is required before the data phase can proceed.
 *   Splitting this into its own named function avoids the classic bug
 *   of a stray `SR1;` statement getting optimized out by the compiler.
 */
static void i2c_clear_addr_flag(I2C_Regs_t *i2c)
{
    volatile uint32_t tmp;
    tmp = i2c->SR1;
    tmp = i2c->SR2;
    (void)tmp;
}

/**
 * @brief Issue a STOP and wait for the bus to become idle.
 *
 * @details
 *   Best-effort — a STOP is queued and BUSY is waited on, but a bus
 *   that's already broken will time out here and the caller proceeds
 *   regardless. This keeps a single failed transfer from permanently
 *   wedging the peripheral; the next I2C_enumInit() can re-arm it.
 */
static void i2c_issue_stop(I2C_Regs_t *i2c)
{
    i2c->CR1 |= I2C_CR1_STOP;
    (void)i2c_wait_flag_clear(i2c, I2C_SR2_BUSY);
}

/*============================================================================*
 *                          GPIO + PERIPHERAL BRING-UP                        *
 *============================================================================*/

/**
 * @brief Configure the two GPIO pins for one I2C channel.
 *
 * @details
 *   Both SCL and SDA are set to alternate-function, open-drain,
 *   high-speed, with the internal pull-up **disabled** — the
 *   I2C bus requires external pull-ups (typically 4.7 kΩ on a
 *   breakout board), and enabling the internal ones as well produces
 *   a value that's too high to matter but non-zero, which is a
 *   confusing thing to leave behind.
 *
 *   The pin numbers match the standard F411 mappings documented in
 *   i2c_private.h's file header.
 */
static ErrorState_t i2c_gpio_init(I2C_Channel_t ch)
{
    GPIO_TypeDef *port_scl = 0;
    GPIO_TypeDef *port_sda = 0;
    uint8_t       pin_scl  = 0U;
    uint8_t       pin_sda  = 0U;

    switch (ch) {
        case I2C_CHANNEL1:
            port_scl = GPIOB; pin_scl = 6U;   /* PB6 = I2C1_SCL */
            port_sda = GPIOB; pin_sda = 7U;   /* PB7 = I2C1_SDA */
            RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
            break;
        case I2C_CHANNEL2:
            port_scl = GPIOB; pin_scl = 10U;  /* PB10 = I2C2_SCL */
            port_sda = GPIOB; pin_sda = 3U;   /* PB3  = I2C2_SDA */
            RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
            break;
        case I2C_CHANNEL3:
            port_scl = GPIOA; pin_scl = 8U;   /* PA8 = I2C3_SCL */
            port_sda = GPIOB; pin_sda = 4U;   /* PB4 = I2C3_SDA */
            RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
            break;
        default:
            return INVALID_PARAM;
    }

    /* Helper macro to reduce repetition — MODER, OTYPER, OSPEEDR, PUPDR,
     * AFR all use the same "clear 2 or 4 bits then OR in the new value"
     * pattern. Local to this function; #undef'd at the end. */
#define CONFIG_I2C_PIN(port, pin, af)                                          \
    do {                                                                       \
        (port)->MODER   &= ~(3U << ((pin) * 2U));                              \
        (port)->MODER   |=  ((uint32_t)GPIO_MODER_AF << ((pin) * 2U));         \
        (port)->OTYPER  |=  (1U << (pin));                    /* open-drain */\
        (port)->OSPEEDR &= ~(3U << ((pin) * 2U));                              \
        (port)->OSPEEDR |=  ((uint32_t)GPIO_OSPEED_HIGH << ((pin) * 2U));      \
        (port)->PUPDR   &= ~(3U << ((pin) * 2U));             /* no internal PU */\
        if ((pin) < 8U) {                                                      \
            (port)->AFR[0] &= ~(0xFU << ((pin) * 4U));                         \
            (port)->AFR[0] |=  ((uint32_t)(af) << ((pin) * 4U));               \
        } else {                                                               \
            (port)->AFR[1] &= ~(0xFU << (((pin) - 8U) * 4U));                  \
            (port)->AFR[1] |=  ((uint32_t)(af) << (((pin) - 8U) * 4U));        \
        }                                                                      \
    } while (0)

    CONFIG_I2C_PIN(port_scl, pin_scl, GPIO_AF_I2C);
    CONFIG_I2C_PIN(port_sda, pin_sda, GPIO_AF_I2C);

#undef CONFIG_I2C_PIN

    return OK;
}

/*============================================================================*
 *                          PUBLIC API — INIT                                 *
 *============================================================================*/

ErrorState_t I2C_enumInit(I2C_Config_t *cfg)
{
    if (cfg == 0) {
        return NULL_POINTER;
    }
    if (cfg->ClockSpeed_Hz == 0U || cfg->PCLK1_Hz == 0U) {
        return INVALID_PARAM;
    }

    I2C_Regs_t *i2c = i2c_base(cfg->Channel);
    if (i2c == 0) {
        return INVALID_PARAM;
    }

    /* 1. Enable the peripheral's APB1 clock. */
    switch (cfg->Channel) {
        case I2C_CHANNEL1: RCC_APB1ENR |= RCC_APB1ENR_I2C1EN; break;
        case I2C_CHANNEL2: RCC_APB1ENR |= RCC_APB1ENR_I2C2EN; break;
        case I2C_CHANNEL3: RCC_APB1ENR |= RCC_APB1ENR_I2C3EN; break;
        default:           return INVALID_PARAM;
    }

    /* 2. GPIO bring-up. */
    ErrorState_t st = i2c_gpio_init(cfg->Channel);
    if (st != OK) {
        return st;
    }

    /* 3. Software reset the peripheral. A stuck I2C block cannot be
     *    recovered by reprogramming registers alone — CR1.SWRST is the
     *    only way out. Clearing it afterwards leaves the peripheral in
     *    the reset state, ready for programming. */
    i2c->CR1 |= I2C_CR1_SWRST;
    i2c->CR1 &= ~I2C_CR1_SWRST;

    /* 4. Program CR2.FREQ = PCLK1 in MHz. This is the peripheral's
     *    internal reference clock for all its timing generators, so it
     *    must be the *real* PCLK1, not a guess. */
    uint32_t freq_mhz = cfg->PCLK1_Hz / 1000000UL;
    if (freq_mhz < 2U)   freq_mhz = 2U;   /* RM0383 minimum */
    if (freq_mhz > 50U)  freq_mhz = 50U;  /* RM0383 maximum */
    i2c->CR2 = (i2c->CR2 & ~I2C_CR2_FREQ_MASK)
             | ((freq_mhz << I2C_CR2_FREQ_SHIFT) & I2C_CR2_FREQ_MASK);

    /* 5. Program CCR — the SCL timing. Two distinct formulas depending
     *    on speed mode (RM0383 §22.6.9):
     *
     *      Standard mode:  Thigh = Tlow = CCR * Tpclk1
     *                      => CCR = PCLK1 / (2 * SCL)
     *
     *      Fast mode, duty 2:1:  Tlow = 2 * Thigh
     *                            => CCR = PCLK1 / (3 * SCL)
     *
     *      Fast mode, duty 16:9: Tlow = (16/9) * Thigh
     *                            => CCR = PCLK1 / (25 * SCL)
     *
     *    All three round-to-nearest by adding half the divisor before
     *    the integer divide. The +1 guards against CCR == 0 if PCLK1
     *    is very low relative to SCL, which shouldn't happen in this
     *    project but costs nothing to defend against.
     */
    uint32_t ccr_val;
    uint32_t ccr_reg = 0U;

    if (cfg->Speed == I2C_SPEED_STANDARD) {
        ccr_val = (cfg->PCLK1_Hz + cfg->ClockSpeed_Hz) / (2U * cfg->ClockSpeed_Hz);
        if (ccr_val < 4U) ccr_val = 4U;   /* RM0383 minimum for standard mode */
    } else {
        /* Fast mode. */
        ccr_reg |= I2C_CCR_FS;
        if (cfg->FM_DutyCycle == I2C_DUTY_16_9) {
            ccr_val = (cfg->PCLK1_Hz + (12U * cfg->ClockSpeed_Hz)) / (25U * cfg->ClockSpeed_Hz);
            ccr_reg |= I2C_CCR_DUTY;
        } else {
            ccr_val = (cfg->PCLK1_Hz + (2U * cfg->ClockSpeed_Hz)) / (3U * cfg->ClockSpeed_Hz);
        }
        if (ccr_val < 1U) ccr_val = 1U;
    }
    i2c->CCR = ccr_reg | (ccr_val & I2C_CCR_CCR_MASK);

    /* 6. Program TRISE. RM0383 §22.6.10 gives the two spec limits in
     *    nanoseconds; converting to PCLK1 cycles and adding 1 gives the
     *    value the register actually wants. The constants in
     *    i2c_private.h are the two "well-known" values from ST's own
     *    examples for a typical PCLK1 — good enough for this project,
     *    and named so the approximation is visible. */
    i2c->TRISE = (cfg->Speed == I2C_SPEED_STANDARD) ? I2C_TRISE_STANDARD
                                                    : I2C_TRISE_FAST;

    /* 7. Own address. Only meaningful in slave mode, but programming
     *    it here means a future slave-mode caller has one fewer
     *    surprise. Bit 14 must be set in 7-bit mode per RM0383. */
    if (cfg->AddressMode == I2C_ADDR_MODE_7BIT) {
        i2c->OAR1 = (1U << 14) | ((uint32_t)cfg->OwnAddress << 1U);
    } else {
        i2c->OAR1 = ((uint32_t)cfg->OwnAddress << 1U);   /* 10-bit: bit14 = 0 */
    }

    /* 8. Enable the peripheral. Everything above is configuration;
     *    nothing takes effect on the wire until PE = 1. */
    i2c->CR1 |= I2C_CR1_PE;

    /* 9. ACK setting. In master mode the hardware NACKs the last byte
     *    of a receive automatically, so this only matters for what
     *    happens to intermediate bytes. Enable if the caller asked. */
    if (cfg->ACK == I2C_ACK_ENABLE) {
        i2c->CR1 |= I2C_CR1_ACK;
    } else {
        i2c->CR1 &= ~I2C_CR1_ACK;
    }

    return OK;
}

/*============================================================================*
 *                          PUBLIC API — TRANSACTIONS                         *
 *============================================================================*/

ErrorState_t I2C_enumMasterTransmit(I2C_Config_t *cfg,
                                    uint8_t addr7,
                                    const uint8_t *data,
                                    uint16_t len)
{
    if (cfg == 0 || (data == 0 && len > 0U)) {
        return NULL_POINTER;
    }
    if (len == 0U) {
        return INVALID_PARAM;
    }

    I2C_Regs_t *i2c = i2c_base(cfg->Channel);
    if (i2c == 0) {
        return INVALID_PARAM;
    }

    /* Clear any sticky error flags from a previous aborted transfer. */
    i2c_clear_error_flags(i2c);

    ErrorState_t st;

    /* START. */
    i2c->CR1 |= I2C_CR1_START;
    st = i2c_wait_flag_set(i2c, I2C_SR1_SB);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    /* Address + W (bit 0 = 0). */
    i2c->DR = (uint8_t)(addr7 << 1U);
    st = i2c_wait_flag_set(i2c, I2C_SR1_ADDR);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }
    i2c_clear_addr_flag(i2c);

    /* Data bytes. Poll TXE before each one; the last byte is special —
     * after writing it we wait for BTF so the byte has actually left
     * the shift register before we issue STOP. */
    for (uint16_t i = 0U; i < len; i++) {
        st = i2c_wait_flag_set(i2c, I2C_SR1_TXE);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->DR = data[i];
    }

    st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    /* STOP. */
    i2c->CR1 |= I2C_CR1_STOP;
    return OK;
}

ErrorState_t I2C_enumMasterReceive(I2C_Config_t *cfg,
                                   uint8_t addr7,
                                   uint8_t *data,
                                   uint16_t len)
{
    if (cfg == 0 || (data == 0 && len > 0U)) {
        return NULL_POINTER;
    }
    if (len == 0U) {
        return INVALID_PARAM;
    }

    I2C_Regs_t *i2c = i2c_base(cfg->Channel);
    if (i2c == 0) {
        return INVALID_PARAM;
    }

    i2c_clear_error_flags(i2c);

    ErrorState_t st;

    /* START. */
    i2c->CR1 |= I2C_CR1_START;
    st = i2c_wait_flag_set(i2c, I2C_SR1_SB);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    /* Address + R (bit 0 = 1). */
    i2c->DR = (uint8_t)((addr7 << 1U) | 0x01U);
    st = i2c_wait_flag_set(i2c, I2C_SR1_ADDR);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    if (len == 1U) {
        /* Single-byte receive: RM0383 §22.6.5 says to clear ACK and set
         * STOP *before* clearing ADDR, so the hardware NACKs and
         * STOPs at the right moment. */
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c->CR1 |=  I2C_CR1_STOP;
        i2c_clear_addr_flag(i2c);

        st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
        if (st != OK) {
            return st;
        }
        data[0] = (uint8_t)(i2c->DR & 0xFFU);
    } else if (len == 2U) {
        /* Two-byte receive: the classic POS + ACK-clear trick. Clearing
         * POS after ADDR lets the hardware NACK after the second byte
         * without an extra read. */
        i2c->CR1 |=  I2C_CR1_POS;
        i2c_clear_addr_flag(i2c);
        i2c->CR1 &= ~I2C_CR1_ACK;

        st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)(i2c->DR & 0xFFU);
        data[1] = (uint8_t)(i2c->DR & 0xFFU);
    } else {
        /* N-byte (N >= 3) receive: ACK every byte until two remain,
         * then clear ACK, wait for BTF, issue STOP, and drain the
         * last two bytes from DR without waiting on RXNE for them. */
        i2c_clear_addr_flag(i2c);

        for (uint16_t i = 0U; i < (uint16_t)(len - 3U); i++) {
            st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
            if (st != OK) {
                i2c_issue_stop(i2c);
                return st;
            }
            data[i] = (uint8_t)(i2c->DR & 0xFFU);
        }

        st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        data[len - 3U] = (uint8_t)(i2c->DR & 0xFFU);

        /* Clear ACK, wait for BTF (two bytes pending in the shift
         * register and DR), then STOP. */
        i2c->CR1 &= ~I2C_CR1_ACK;
        st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->CR1 |= I2C_CR1_STOP;

        data[len - 2U] = (uint8_t)(i2c->DR & 0xFFU);
        data[len - 1U] = (uint8_t)(i2c->DR & 0xFFU);
    }

    /* Restore ACK for the next transaction. */
    if (cfg->ACK == I2C_ACK_ENABLE) {
        i2c->CR1 |= I2C_CR1_ACK;
    }

    return OK;
}

ErrorState_t I2C_enumMasterTransmitReceive(I2C_Config_t *cfg,
                                           uint8_t addr7,
                                           const uint8_t *tx,
                                           uint16_t txlen,
                                           uint8_t *rx,
                                           uint16_t rxlen)
{
    if (cfg == 0) {
        return NULL_POINTER;
    }
    if ((tx == 0 && txlen > 0U) || (rx == 0 && rxlen > 0U)) {
        return NULL_POINTER;
    }
    if (txlen == 0U && rxlen == 0U) {
        return INVALID_PARAM;
    }

    /* Degenerate cases: no repeated START needed. */
    if (rxlen == 0U) {
        return I2C_enumMasterTransmit(cfg, addr7, tx, txlen);
    }
    if (txlen == 0U) {
        return I2C_enumMasterReceive(cfg, addr7, rx, rxlen);
    }

    I2C_Regs_t *i2c = i2c_base(cfg->Channel);
    if (i2c == 0) {
        return INVALID_PARAM;
    }

    i2c_clear_error_flags(i2c);

    ErrorState_t st;

    /* Phase 1: write bytes. Identical to MasterTransmit up to the
     * point where a STOP would be issued — we deliberately don't. */
    i2c->CR1 |= I2C_CR1_START;
    st = i2c_wait_flag_set(i2c, I2C_SR1_SB);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    i2c->DR = (uint8_t)(addr7 << 1U);
    st = i2c_wait_flag_set(i2c, I2C_SR1_ADDR);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }
    i2c_clear_addr_flag(i2c);

    for (uint16_t i = 0U; i < txlen; i++) {
        st = i2c_wait_flag_set(i2c, I2C_SR1_TXE);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->DR = tx[i];
    }

    st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    /* Phase 2: repeated START, then read. */
    i2c->CR1 |= I2C_CR1_START;
    st = i2c_wait_flag_set(i2c, I2C_SR1_SB);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    i2c->DR = (uint8_t)((addr7 << 1U) | 0x01U);
    st = i2c_wait_flag_set(i2c, I2C_SR1_ADDR);
    if (st != OK) {
        i2c_issue_stop(i2c);
        return st;
    }

    /* Same three cases as MasterReceive. */
    if (rxlen == 1U) {
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c->CR1 |=  I2C_CR1_STOP;
        i2c_clear_addr_flag(i2c);

        st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
        if (st != OK) {
            return st;
        }
        rx[0] = (uint8_t)(i2c->DR & 0xFFU);
    } else if (rxlen == 2U) {
        i2c->CR1 |=  I2C_CR1_POS;
        i2c_clear_addr_flag(i2c);
        i2c->CR1 &= ~I2C_CR1_ACK;

        st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->CR1 |= I2C_CR1_STOP;
        rx[0] = (uint8_t)(i2c->DR & 0xFFU);
        rx[1] = (uint8_t)(i2c->DR & 0xFFU);
    } else {
        i2c_clear_addr_flag(i2c);

        for (uint16_t i = 0U; i < (uint16_t)(rxlen - 3U); i++) {
            st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
            if (st != OK) {
                i2c_issue_stop(i2c);
                return st;
            }
            rx[i] = (uint8_t)(i2c->DR & 0xFFU);
        }

        st = i2c_wait_flag_set(i2c, I2C_SR1_RXNE);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        rx[rxlen - 3U] = (uint8_t)(i2c->DR & 0xFFU);

        i2c->CR1 &= ~I2C_CR1_ACK;
        st = i2c_wait_flag_set(i2c, I2C_SR1_BTF);
        if (st != OK) {
            i2c_issue_stop(i2c);
            return st;
        }
        i2c->CR1 |= I2C_CR1_STOP;

        rx[rxlen - 2U] = (uint8_t)(i2c->DR & 0xFFU);
        rx[rxlen - 1U] = (uint8_t)(i2c->DR & 0xFFU);
    }

    /* Restore ACK for the next transaction. */
    if (cfg->ACK == I2C_ACK_ENABLE) {
        i2c->CR1 |= I2C_CR1_ACK;
    }

    return OK;
}
