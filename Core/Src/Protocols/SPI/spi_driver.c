/**
 ******************************************************************************
 * @file    spi_driver.c
 * @brief   Implementation of the bare-metal SPI1 master protocol layer
 *          declared in spi_driver.h.
 * @details
 *   Register-level driver for STM32F411 SPI1, written against RM0383
 *   §28 ("Serial peripheral interface"). No HAL_* calls, no higher-layer
 *   dependency.
 *
 *   Bus topology assumed on the Black Pill:
 *     SPI1  PA5 (SCK) / PA6 (MISO) / PA7 (MOSI)   AF5
 *     CS    PA3 (ICM20948), PA4 (BMP388)          manual GPIO, active low
 *
 *   SCK/MISO/MOSI are fixed by the STM32F411's alternate-function map —
 *   AF5 on GPIOA is the only pin set that exposes SPI1 without a remap.
 *   Chip-select is per-sensor and driven manually as plain GPIO, because
 *   the ICM20948 and the BMP388 share the clock and data lines and each
 *   needs its own CS.
 *
 *   Timing: at 100 MHz SYSCLK with APB2 undivided, APB2 = 100 MHz. With
 *   BR = DIV16 the SCK line runs at 6.25 MHz, which is under both the
 *   ICM20948's 7 MHz limit and the BMP388's 10 MHz limit. SPI mode is 0
 *   (CPOL = 0, CPHA = 0), which both parts accept.
 *
 *   There is no static or global mutable state in this file. All
 *   peripheral state lives in the SPI1 registers themselves, and all
 *   chip-select state lives in the GPIOA ODR/BSRR registers.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#include "spi_driver.h"

/*============================================================================*
 *                          PERIPHERAL BASE ADDRESSES                         *
 *============================================================================*/

#define PERIPH_BASE         0x40000000UL
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000UL)
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)

#define RCC_BASE            (AHB1PERIPH_BASE + 0x3800UL)
#define GPIOA_BASE          (AHB1PERIPH_BASE + 0x0000UL)
#define SPI1_BASE           (APB2PERIPH_BASE + 0x3000UL)

/*============================================================================*
 *                          REGISTER LAYOUTS                                  *
 *============================================================================*/

/**
 * @brief GPIOA register layout (RM0383 §8.4).
 */
typedef struct {
    volatile uint32_t MODER;    /**< 0x00 Mode register. */
    volatile uint32_t OTYPER;   /**< 0x04 Output type. */
    volatile uint32_t OSPEEDR;  /**< 0x08 Output speed. */
    volatile uint32_t PUPDR;    /**< 0x0C Pull-up/pull-down. */
    volatile uint32_t IDR;      /**< 0x10 Input data. */
    volatile uint32_t ODR;      /**< 0x14 Output data. */
    volatile uint32_t BSRR;     /**< 0x18 Bit set/reset. */
    volatile uint32_t LCKR;     /**< 0x1C Lock. */
    volatile uint32_t AFR[2];   /**< 0x20/0x24 Alternate function low/high. */
} GPIO_TypeDef;

/**
 * @brief SPI1 register layout (RM0383 §28.5).
 */
typedef struct {
    volatile uint32_t CR1;      /**< 0x00 Control 1. */
    volatile uint32_t CR2;      /**< 0x04 Control 2. */
    volatile uint32_t SR;       /**< 0x08 Status. */
    volatile uint32_t DR;       /**< 0x0C Data. */
    volatile uint32_t CRCPR;    /**< 0x10 CRC polynomial. */
    volatile uint32_t RXCRCR;   /**< 0x14 RX CRC. */
    volatile uint32_t TXCRCR;   /**< 0x18 TX CRC. */
    volatile uint32_t I2SCFGR;  /**< 0x1C I2S config (unused). */
    volatile uint32_t I2SPR;    /**< 0x20 I2S prescaler (unused). */
} SPI_TypeDef;

/**
 * @brief Just enough of RCC to enable the two clocks we need.
 */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t          RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t          RESERVED1[2];
    volatile uint32_t AHB1ENR;   /**< +0x30 AHB1 peripheral clock enable. */
} RCC_TypeDef;

#define GPIOA   ((GPIO_TypeDef *)GPIOA_BASE)
#define SPI1    ((SPI_TypeDef  *)SPI1_BASE)
#define RCC     ((RCC_TypeDef  *)RCC_BASE)

/*============================================================================*
 *                          RCC REGISTER BITS                                 *
 *============================================================================*/

#define RCC_AHB1ENR_GPIOAEN   (1U << 0)    /**< GPIOA clock enable. */

/** @brief APB2 peripheral clock enable register (RCC + 0x44).
 *         Non-contiguous in the RCC struct above, so accessed by
 *         absolute offset rather than as a struct field. */
#define RCC_APB2ENR          (*(volatile uint32_t *)(RCC_BASE + 0x44UL))
#define RCC_APB2ENR_SPI1EN   (1U << 12)    /**< SPI1 clock enable. */

/*============================================================================*
 *                          SPI CR1 BITS                                      *
 *============================================================================*/

#define SPI_CR1_CPHA         (1U << 0)     /**< Clock phase: 0 = first edge. */
#define SPI_CR1_CPOL         (1U << 1)     /**< Clock polarity: 0 = idle low. */
#define SPI_CR1_MSTR         (1U << 2)     /**< Master mode. */
/** @brief Baud-rate prescaler field, bits[5:3]. 000=/2, 001=/4, 010=/8,
 *         011=/16, 100=/32, 101=/64, 110=/128, 111=/256. */
#define SPI_CR1_BR_DIV16     (3U << 3)     /**< APB2 / 16 = 6.25 MHz at
                                                100 MHz APB2. */
#define SPI_CR1_SPE          (1U << 6)     /**< SPI enable. */
#define SPI_CR1_LSBFIRST     (1U << 7)     /**< LSB-first (unused; MSB is
                                                the default and what both
                                                sensors expect). */
#define SPI_CR1_SSI          (1U << 8)     /**< Internal slave select: kept
                                                high in software-NSS mode. */
#define SPI_CR1_SSM          (1U << 9)     /**< Software slave management:
                                                NSS is controlled by SSI,
                                                not by the NSS pin. */
#define SPI_CR1_DFF          (1U << 11)    /**< Data frame format: 0 = 8-bit,
                                                1 = 16-bit. Both sensors
                                                use 8-bit. */

/*============================================================================*
 *                          SPI SR BITS                                       *
 *============================================================================*/

#define SPI_SR_RXNE          (1U << 0)     /**< RX buffer not empty. */
#define SPI_SR_TXE           (1U << 1)     /**< TX buffer empty. */
#define SPI_SR_BSY           (1U << 7)     /**< Busy: a transfer is in
                                                progress. */

/*============================================================================*
 *                          GPIO MODER / AF CONSTANTS                         *
 *============================================================================*/

#define GPIO_MODER_OUTPUT    1U            /**< MODER field: general-purpose
                                                output. */
#define GPIO_MODER_AF        2U            /**< MODER field: alternate
                                                function. */
#define GPIO_OSPEED_HIGH     2U            /**< OSPEEDR field: high speed
                                                (up to 50 MHz on this part).*/
#define GPIO_OSPEED_VHIGH    3U            /**< OSPEEDR field: very high
                                                (up to 100 MHz). */
#define GPIO_AF_SPI1         5U            /**< AF5 = SPI1 on PA5/6/7. */

/*============================================================================*
 *                          PIN CONFIGURATION                                 *
 *============================================================================*/

/**
 * @brief One-time GPIOA bring-up: SCK/MISO/MOSI as AF5, CS pins as
 *        outputs idle high.
 *
 * @details
 *   Sequence:
 *     1. Enable the GPIOA clock.
 *     2. Configure PA5/PA6/PA7 as alternate-function push-pull, AF5,
 *        very-high speed. These three pins are fixed by the silicon —
 *        AF5 on GPIOA is the only pin set that exposes SPI1 on this
 *        part without a remap.
 *     3. Configure PA3 and PA4 (the two default CS pins) as plain
 *        push-pull outputs, and drive them **high**. This is the piece
 *        of the bring-up that matters most: both sensors' SPI state
 *        machines require CS to be deasserted (high) before their first
 *        clock edge, and a floating CS at power-on reads as a random
 *        value, which can leave the BMP388 mid-transaction on the very
 *        first access.
 *
 *   Called exactly once, from SPI1_Init(). Idempotent.
 *
 * @warning Configures **both** PA3 and PA4 as CS pins regardless of
 *          which sensor drivers are actually built into the binary. If
 *          PA3 is used for something else (e.g. USART2_RX for the GPS
 *          module), that peripheral's own bring-up must run **after**
 *          this one, or this function will overwrite its GPIO config.
 *          The safest rule is: no other peripheral uses PA3 or PA4 on
 *          this project.
 */
static void SPI1_GPIO_Init(void)
{
    /* 1. GPIOA clock. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* 2. SCK / MISO / MOSI: alternate-function, push-pull, AF5,
     *    very-high speed. */
    GPIOA->MODER &= ~(  (3U << (SPI1_SCK_PIN  * 2U))
                      | (3U << (SPI1_MISO_PIN * 2U))
                      | (3U << (SPI1_MOSI_PIN * 2U)) );
    GPIOA->MODER |=  (  ((uint32_t)GPIO_MODER_AF << (SPI1_SCK_PIN  * 2U))
                      | ((uint32_t)GPIO_MODER_AF << (SPI1_MISO_PIN * 2U))
                      | ((uint32_t)GPIO_MODER_AF << (SPI1_MOSI_PIN * 2U)) );

    /* AFR[0] covers pins 0..7, which is all three of ours. Clear then
     * set, in that order — the clear is essential or the previous AF
     * value ORs with the new one. */
    GPIOA->AFR[0] &= ~(  (0xFU << (SPI1_SCK_PIN  * 4U))
                       | (0xFU << (SPI1_MISO_PIN * 4U))
                       | (0xFU << (SPI1_MOSI_PIN * 4U)) );
    GPIOA->AFR[0] |=  (  ((uint32_t)GPIO_AF_SPI1 << (SPI1_SCK_PIN  * 4U))
                       | ((uint32_t)GPIO_AF_SPI1 << (SPI1_MISO_PIN * 4U))
                       | ((uint32_t)GPIO_AF_SPI1 << (SPI1_MOSI_PIN * 4U)) );

    /* Output speed: very-high on all three, so the 6.25 MHz SCK has
     * clean edges without a separate timing analysis. */
    GPIOA->OSPEEDR &= ~(  (3U << (SPI1_SCK_PIN  * 2U))
                        | (3U << (SPI1_MISO_PIN * 2U))
                        | (3U << (SPI1_MOSI_PIN * 2U)) );
    GPIOA->OSPEEDR |=  (  ((uint32_t)GPIO_OSPEED_VHIGH << (SPI1_SCK_PIN  * 2U))
                        | ((uint32_t)GPIO_OSPEED_VHIGH << (SPI1_MISO_PIN * 2U))
                        | ((uint32_t)GPIO_OSPEED_VHIGH << (SPI1_MOSI_PIN * 2U)) );

    /* No pull-ups on the SPI lines — both sensors' breakouts (or the
     * board itself) already provide whatever the wiring needs, and the
     * internal pull-ups are too weak to matter for a 6 MHz bus. */
    GPIOA->PUPDR &= ~(  (3U << (SPI1_SCK_PIN  * 2U))
                      | (3U << (SPI1_MISO_PIN * 2U))
                      | (3U << (SPI1_MOSI_PIN * 2U)) );

    /* 3. CS pins: general-purpose output, push-pull, high speed, idle
     *    high. The order matters — set the MODER to output *before*
     *    writing BSRR, or the write lands in a config that doesn't yet
     *    drive the pin. */
    GPIOA->MODER &= ~(  (3U << (SPI1_CS_ICM20948_PIN * 2U))
                      | (3U << (SPI1_CS_BMP388_PIN   * 2U)) );
    GPIOA->MODER |=  (  ((uint32_t)GPIO_MODER_OUTPUT << (SPI1_CS_ICM20948_PIN * 2U))
                      | ((uint32_t)GPIO_MODER_OUTPUT << (SPI1_CS_BMP388_PIN   * 2U)) );

    GPIOA->OTYPER &= ~(  (1U << SPI1_CS_ICM20948_PIN)
                       | (1U << SPI1_CS_BMP388_PIN) );

    GPIOA->OSPEEDR &= ~(  (3U << (SPI1_CS_ICM20948_PIN * 2U))
                        | (3U << (SPI1_CS_BMP388_PIN   * 2U)) );
    GPIOA->OSPEEDR |=  (  ((uint32_t)GPIO_OSPEED_HIGH << (SPI1_CS_ICM20948_PIN * 2U))
                        | ((uint32_t)GPIO_OSPEED_HIGH << (SPI1_CS_BMP388_PIN   * 2U)) );

    GPIOA->PUPDR &= ~(  (3U << (SPI1_CS_ICM20948_PIN * 2U))
                      | (3U << (SPI1_CS_BMP388_PIN   * 2U)) );

    /* Both CS lines idle HIGH before the pins are driven. */
    GPIOA->BSRR = (1U << SPI1_CS_ICM20948_PIN)
                | (1U << SPI1_CS_BMP388_PIN);
}

/*============================================================================*
 *                          PUBLIC API — INIT                                 *
 *============================================================================*/

void SPI1_Init(void)
{
    /* 1. GPIO + peripheral clock. */
    SPI1_GPIO_Init();
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* 2. Program CR1. Writing the whole register at once (rather than
     *    OR-ing into whatever was there) makes the mode an explicit
     *    decision rather than a side effect of whatever ran before. */
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR           /* master */
               | SPI_CR1_SSM            /* software NSS: NSS is internal */
               | SPI_CR1_SSI            /* internal NSS held high */
               | SPI_CR1_BR_DIV16;      /* APB2 / 16 = 6.25 MHz SCK */

    /* CPOL = 0, CPHA = 0 (mode 0) is the reset state of these bits —
     * explicitly not set, so no code needed. Both the ICM20948 and the
     * BMP388 accept mode 0. */

    /* 3. Enable the peripheral. Everything above is configuration;
     *    nothing takes effect on the wire until SPE = 1. */
    SPI1->CR1 |= SPI_CR1_SPE;
}

/*============================================================================*
 *                          PUBLIC API — CHIP SELECT                          *
 *============================================================================*/

void SPI1_CS_Enable(uint8_t cs_pin)
{
    /* BSRR: the upper 16 bits reset (drive low) the corresponding pin.
     * One write, no read-modify-write, so an interrupt between two CS
     * operations on different pins cannot corrupt either of them. */
    GPIOA->BSRR = (1U << (cs_pin + 16U));
}

void SPI1_CS_Disable(uint8_t cs_pin)
{
    /* BSRR: the lower 16 bits set (drive high) the corresponding pin. */
    GPIOA->BSRR = (1U << cs_pin);
}

/*============================================================================*
 *                          PUBLIC API — TRANSFER                             *
 *============================================================================*/

uint8_t SPI1_TransferByte(uint8_t data)
{
    /* Wait for TXE before writing DR — the shift register must be empty
     * or the write overwrites the previous byte. */
    while (!(SPI1->SR & SPI_SR_TXE)) {
        /* spin */
    }
    SPI1->DR = data;

    /* Wait for RXNE — a byte has been clocked in during the same burst
     * that clocked our byte out. */
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        /* spin */
    }
    uint8_t received = (uint8_t)(SPI1->DR & 0xFFU);

    /* Wait for BSY to clear before returning. This matters for the CS
     * timing: if the caller deasserts CS while the peripheral is still
     * busy, some slaves miss the last clock edge. */
    while (SPI1->SR & SPI_SR_BSY) {
        /* spin */
    }

    return received;
}

void SPI1_WriteBuffer(const uint8_t *data, uint16_t length)
{
    if (data == 0) {
        return;
    }
    for (uint16_t i = 0U; i < length; i++) {
        (void)SPI1_TransferByte(data[i]);
    }
}

void SPI1_ReadBuffer(uint8_t *buffer, uint16_t length)
{
    if (buffer == 0) {
        return;
    }
    for (uint16_t i = 0U; i < length; i++) {
        /* 0xFF is the conventional dummy byte — any value works, but
         * using 0xFF consistently makes a logic-analyzer capture easy
         * to read (dummy bytes are visibly distinct from real data). */
        buffer[i] = SPI1_TransferByte(0xFFU);
    }
}

/*============================================================================*
 *                          PUBLIC API — REGISTER ACCESS                      *
 *============================================================================*/

void SPI1_ReadRegisters(uint8_t cs_pin,
                        uint8_t reg_addr,
                        uint8_t *buffer,
                        uint16_t length)
{
    if (buffer == 0 && length > 0U) {
        return;
    }

    SPI1_CS_Enable(cs_pin);
    (void)SPI1_TransferByte(reg_addr);
    SPI1_ReadBuffer(buffer, length);
    SPI1_CS_Disable(cs_pin);
}

void SPI1_WriteRegister(uint8_t cs_pin,
                        uint8_t reg_addr,
                        uint8_t value)
{
    SPI1_CS_Enable(cs_pin);
    (void)SPI1_TransferByte(reg_addr);
    (void)SPI1_TransferByte(value);
    SPI1_CS_Disable(cs_pin);
}
