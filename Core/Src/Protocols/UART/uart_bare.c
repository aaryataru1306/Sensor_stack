/**
 ******************************************************************************
 * @file    uart_bare.c
 * @brief   Implementation of the bare-metal UART/USART protocol layer
 *          declared in uart_bare.h.
 * @details
 *   Register-level driver for STM32F411 USART1 / USART2 / USART6, written
 *   against RM0383 §19. No HAL_* calls, no higher-layer dependency.
 *
 *   The handle-based API means two instances can coexist trivially: the
 *   GPS module on USART2 and a debug console on USART1 are the classic
 *   pairing on this board, and both can be driven by two separate
 *   UART_Bare_Handle_t instances with no shared state between them.
 *
 *   RCC clock enable and GPIO pin configuration are deliberately **not**
 *   done by this file. Which pins a given USART comes out on is a board
 *   decision, and it lives one level up, in the platform bus file
 *   (gps_bus_stm32.c) — exactly as documented in uart_bare.h's
 *   UART_Bare_Init() comment. This file only knows how to talk to the
 *   USART register block once it has been clocked and its pins are
 *   already configured.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#include "uart_bare.h"

/*============================================================================*
 *                          REGISTER LAYOUT                                   *
 *============================================================================*/

/**
 * @brief One USART peripheral's register block (RM0383 §19.6).
 *
 * @details
 *   Byte-for-byte identical for USART1, USART2 and USART6 — a single
 *   struct definition is enough, and the handle's base pointer is cast
 *   to this type at every access.
 */
typedef struct {
    volatile uint32_t SR;    /**< 0x00 Status register. */
    volatile uint32_t DR;    /**< 0x04 Data register. */
    volatile uint32_t BRR;   /**< 0x08 Baud rate register. */
    volatile uint32_t CR1;   /**< 0x0C Control register 1. */
    volatile uint32_t CR2;   /**< 0x10 Control register 2. */
    volatile uint32_t CR3;   /**< 0x14 Control register 3. */
    volatile uint32_t GTPR;  /**< 0x18 Guard time and prescaler. */
} USART_Regs_t;

/*============================================================================*
 *                          SR — STATUS REGISTER BITS                         *
 *============================================================================*/

#define USART_SR_PE      (1U << 0)    /**< Parity error. */
#define USART_SR_FE      (1U << 1)    /**< Framing error: stop bit missing —
                                           usually a baud-rate mismatch. */
#define USART_SR_NE      (1U << 2)    /**< Noise error. */
#define USART_SR_ORE     (1U << 3)    /**< Overrun error: a byte arrived
                                           before the previous one was
                                           read; the older byte is lost. */
#define USART_SR_RXNE    (1U << 5)    /**< RX not empty: a byte is waiting
                                           in DR. */
#define USART_SR_TC      (1U << 6)    /**< Transmission complete: the last
                                           byte has fully shifted out. */
#define USART_SR_TXE     (1U << 7)    /**< TX empty: DR is ready for the
                                           next byte. */

/*============================================================================*
 *                          CR1 — CONTROL REGISTER 1 BITS                     *
 *============================================================================*/

#define USART_CR1_RE     (1U << 2)    /**< Receiver enable. */
#define USART_CR1_TE     (1U << 3)    /**< Transmitter enable. */
#define USART_CR1_UE     (1U << 13)   /**< USART enable. */

/*============================================================================*
 *                          PUBLIC API — BRR                                  *
 *============================================================================*/

uint32_t UART_Bare_ComputeBRR(uint32_t pclk, uint32_t baud)
{
    /* The STM32 USART's baud-rate generator wants:
     *
     *     USARTDIV = PCLK / (16 * baud)
     *     BRR      = round(USARTDIV * 16)
     *
     * which algebraically reduces to round(PCLK / baud). The half-baud
     * add below is what turns a truncating integer divide into a
     * round-to-nearest — important because a 1-bit error in the BRR
     * value at 115200 baud is the difference between a working link and
     * one that framing-errors on every byte. */
    if (baud == 0U) {
        return 0U;
    }
    return (pclk + (baud / 2U)) / baud;
}

/*============================================================================*
 *                          PUBLIC API — INIT                                 *
 *============================================================================*/

UART_Bare_Status_t UART_Bare_Init(UART_Bare_Handle_t *handle,
                                  volatile uint32_t *base,
                                  const UART_Bare_Config_t *cfg)
{
    if (handle == 0 || base == 0 || cfg == 0) {
        return UART_BARE_ERROR_NOT_INITIALIZED;
    }
    if (cfg->peripheral_clock_hz == 0U || cfg->baud_rate == 0U) {
        return UART_BARE_ERROR_NOT_INITIALIZED;
    }

    /* Store base and config in the handle before touching registers —
     * if the register writes below fail for any reason, the handle is
     * at least in a state a debugger can inspect. */
    handle->base = base;
    handle->cfg  = *cfg;
    handle->initialized = 0U;

    USART_Regs_t *u = (USART_Regs_t *)base;

    /* 1. Disable the peripheral before reconfiguring. Programming BRR
     *    while UE = 1 is explicitly disallowed by RM0383 (§19.6.5
     *    "BRR ... must not be written while UE = 1"). Since this
     *    function is idempotent, a re-init has to be safe on an already
     *    running peripheral — so clear UE first, then reprogram. */
    u->CR1 = 0U;

    /* 2. Baud rate. */
    u->BRR = UART_Bare_ComputeBRR(cfg->peripheral_clock_hz,
                                  cfg->baud_rate) & 0xFFFFU;

    /* 3. Enable the peripheral, transmitter and receiver. 8N1 is the
     *    reset state of CR2 and CR3 — no code needed. */
    u->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    handle->initialized = 1U;
    return UART_BARE_OK;
}

/*============================================================================*
 *                          PUBLIC API — RECEIVE                              *
 *============================================================================*/

UART_Bare_Status_t UART_Bare_ReceiveByte(UART_Bare_Handle_t *handle,
                                         uint8_t *out)
{
    if (handle == 0 || out == 0 || !handle->initialized) {
        return UART_BARE_ERROR_NOT_INITIALIZED;
    }

    USART_Regs_t *u = (USART_Regs_t *)handle->base;

    /* Order matters: ORE / FE / NE are examined *before* the RXNE check,
     * because reading DR clears all four flags. If any of the three
     * error flags is set, the byte currently in DR came in with a
     * problem and is not worth handing to the caller — report the
     * error instead. The caller can then decide whether to drop the
     * byte, log the fault, or reset the link.
     *
     * Reading DR on the error path is what actually clears the flags;
     * the read result is discarded. */
    if (u->SR & USART_SR_ORE) {
        (void)u->DR;
        return UART_BARE_ERROR_OVERRUN;
    }
    if (u->SR & USART_SR_FE) {
        (void)u->DR;
        return UART_BARE_ERROR_FRAMING;
    }
    if (u->SR & USART_SR_NE) {
        (void)u->DR;
        return UART_BARE_ERROR_NOISE;
    }

    if (!(u->SR & USART_SR_RXNE)) {
        /* The routine "no byte waiting" case — not an error. */
        return UART_BARE_ERROR_NO_DATA;
    }

    /* 8N1: the low 8 bits of DR hold the byte, the upper bits are
     * reserved in this mode. */
    *out = (uint8_t)(u->DR & 0xFFU);
    return UART_BARE_OK;
}

/*============================================================================*
 *                          PUBLIC API — TRANSMIT                             *
 *============================================================================*/

UART_Bare_Status_t UART_Bare_Transmit(UART_Bare_Handle_t *handle,
                                      const uint8_t *data,
                                      uint16_t length)
{
    if (handle == 0 || !handle->initialized) {
        return UART_BARE_ERROR_NOT_INITIALIZED;
    }
    if (data == 0 && length > 0U) {
        return UART_BARE_ERROR_NOT_INITIALIZED;
    }
    if (length == 0U) {
        /* Nothing to send — success by definition. */
        return UART_BARE_OK;
    }

    USART_Regs_t *u = (USART_Regs_t *)handle->base;
    uint32_t timeout_iters = handle->cfg.timeout_iters;

    /* Per-byte TXE poll. The timeout counter is per-byte, not shared
     * across the whole transfer: a slow-but-progressing transfer is
     * not a fault, and a shared budget would false-positive on a long
     * message at low baud. A per-byte budget fails only when the wire
     * is genuinely stuck (dead peripheral, no clock, SDA shorted). */
    for (uint16_t i = 0U; i < length; i++) {
        uint32_t spin = timeout_iters;
        while (!(u->SR & USART_SR_TXE)) {
            if (spin-- == 0U) {
                return UART_BARE_ERROR_TIMEOUT;
            }
        }
        u->DR = data[i];
    }

    /* After the last byte is written, TXE goes high immediately but the
     * byte is still shifting out. The caller may want to know the line
     * is idle before returning (e.g. before tearing down a bus, or
     * before powering off a module), so wait for TC as well — with the
     * same per-byte budget. */
    uint32_t spin = timeout_iters;
    while (!(u->SR & USART_SR_TC)) {
        if (spin-- == 0U) {
            return UART_BARE_ERROR_TIMEOUT;
        }
    }

    return UART_BARE_OK;
}
