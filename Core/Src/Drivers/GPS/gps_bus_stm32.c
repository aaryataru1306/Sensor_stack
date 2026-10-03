/**
 * @file    gps_bus_stm32.c
 * @brief   Bare-metal (zero HAL_*) implementation of the GPS_Bus contract
 *          for real STM32 hardware, built directly on
 *          Drivers/UART/uart_bare.c — the register-level USART driver
 *          Member C owns from scratch (Phase2_BareMetal_Task_Division.pdf
 *          §"Member C — ... Bare-metal USART for GPS/GNSS (this is new —
 *          nobody else owns UART)").
 *
 * @details Gated behind `GPS_TARGET_STM32`, exactly like
 *          `bmp388_bus_stm32.c`'s `BMP388_TARGET_STM32` gate — so it
 *          compiles to nothing and is automatically excluded from the
 *          host test build (tests/qemu_gps links mock_gps_bus.c instead)
 *          without any Makefile-side `#ifdef` bookkeeping.
 *
 *          **Why this file is bare-metal and bmp388_bus_stm32.c /
 *          icm42688_bus_stm32.c are not:** those two were written before
 *          this task existed, against ST's HAL (`HAL_SPI_Transmit`
 *          etc.) — acceptable under the *first* Phase 2 task document's
 *          looser "no ST HAL or external libraries" framing applied
 *          loosely, but not under Phase2_BareMetal_Task_Division.pdf's
 *          stricter, explicit "zero `HAL_*` calls" rule. This file
 *          follows the stricter rule, since it's the one this task
 *          actually specifies. Bringing the two SPI/I2C bus files in
 *          line with the same standard is real, valuable follow-up work
 *          — flagged here and in the repo root README's "Known
 *          simplifications" — but it's Members A/B's files to change,
 *          not bundled into this delivery uninvited.
 *
 * @note    Wiring assumption: GPS/GNSS module on USART2 (TX = PA2,
 *          RX = PA3 on every STM32F1/F4 part that exposes USART2 on
 *          GPIOA, which is the default AF mapping needing no remap) —
 *          matching the original task document's "Uses a UART port (e.g.
 *          USART2)" and leaving USART1 free for a debug console, exactly
 *          as the existing QEMU boot demo already uses USART1 for its
 *          own banner output.
 */

#include "gps_bus.h"

#if defined(GPS_TARGET_STM32)

#include "uart_bare.h"
#include <stddef.h>

/** @brief USART2 base address — identical on STM32F100 (this project's
 *         QEMU backend target) and every STM32F1/F4 part; RM0008 Table 3
 *         / RM0090 Table 1. */
#define USART2_BASE ((volatile uint32_t *)0x40004400UL)

/** @brief One hard-wired instance, matching how the existing QEMU boot
 *         demo also assumes exactly one IMU instance (icm42688_bus_
 *         baremetal_qemu.c's cs_low()/cs_high()) — this project has no
 *         multi-instance GPS use case, so a single static handle (not a
 *         global driver-state variable — this is bus-plumbing, the same
 *         category as a fixed peripheral base address, not sensor state;
 *         DRIVER_STANDARD.md Section 2's "no static/global mutable
 *         state" rule targets the driver's *own* state, which lives
 *         entirely in the caller-supplied GPS_Handle_t) is the simplest
 *         correct answer here. */
static UART_Bare_Handle_t s_gps_uart;

/**
 * @brief One-time USART2 + GPIOA bring-up. Call before GPS_Init().
 * @details RCC/GPIO sequencing lives here, one level below
 *          uart_bare.c, exactly as uart_bare.h's own header comment
 *          describes: the peripheral driver stays chip-family-agnostic,
 *          and each backend's bus file does its own clock/pin bring-up.
 * @param   peripheral_clock_hz  APB1 clock actually feeding USART2 on
 *                                the target board (varies by clock-tree
 *                                configuration — pass the real value,
 *                                not a guess; see UART_Bare_ComputeBRR()'s
 *                                comment for why this matters).
 * @param   baud_rate            Module's configured baud rate (9600 is
 *                                the near-universal power-on default —
 *                                see gps_bus.h and the task brief).
 */
void gps_bus_stm32_setup(uint32_t peripheral_clock_hz, uint32_t baud_rate)
{
    #define RCC_BASE     0x40021000UL
    #define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x1CUL))
    #define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x18UL))
    #define RCC_APB1ENR_USART2EN (1UL << 17)
    #define RCC_APB2ENR_IOPAEN   (1UL << 2)

    #define GPIOA_BASE   0x40010800UL
    #define GPIOA_CRL    (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = USART2_TX: alternate-function push-pull, 50 MHz (CRL bits
     * [11:8] = 0xB). PA3 = USART2_RX: input floating (CRL bits [15:12] =
     * 0x4). CRL covers pins 0-7, so PA2/PA3 are its middle nibbles —
     * RM0008 §9.2.2. */
    GPIOA_CRL &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOA_CRL |=  ((0xBUL << 8) | (0x4UL << 12));

    UART_Bare_Config_t config = {
        .peripheral_clock_hz = peripheral_clock_hz,
        .baud_rate           = baud_rate,
        .timeout_iters       = 1000000U,
    };
    (void)UART_Bare_Init(&s_gps_uart, USART2_BASE, &config);

    #undef RCC_BASE
    #undef RCC_APB1ENR
    #undef RCC_APB2ENR
    #undef RCC_APB1ENR_USART2EN
    #undef RCC_APB2ENR_IOPAEN
    #undef GPIOA_BASE
    #undef GPIOA_CRL
}

/*============================================================================*
 *                     GPS_Bus CONTRACT IMPLEMENTATION                        *
 *============================================================================*/

int GPS_BusReadByte(void *bus_context, uint8_t *byte_out)
{
    (void)bus_context; /* single hard-wired instance — see file header */
    UART_Bare_Status_t st = UART_Bare_ReceiveByte(&s_gps_uart, byte_out);
    switch (st) {
        case UART_BARE_OK:            return 0;
        case UART_BARE_ERROR_NO_DATA: return 1;
        default:                      return -1; /* overrun/framing/noise/
                                                       not-initialized all
                                                       collapse to "hard
                                                       fault" at this
                                                       contract's
                                                       granularity — see
                                                       gps_bus.h */
    }
}

int GPS_BusWrite(void *bus_context, const uint8_t *data, uint16_t length)
{
    (void)bus_context;
    if (data == NULL && length > 0U) {
        return -1;
    }
    UART_Bare_Status_t st = UART_Bare_Transmit(&s_gps_uart, data, length);
    return (st == UART_BARE_OK) ? 0 : -1;
}

void GPS_DelayMs(uint32_t ms)
{
    /* No SysTick/DWT delay is wired up anywhere in this project yet (the
     * QEMU boot demo's own ICM42688_DelayMs() is an uncalibrated busy
     * loop for the same reason — see backends/qemu-cortex-m/README.md).
     * Phase2_BareMetal_Task_Division.pdf's coordination section
     * anticipates exactly this: "Member A builds one delay_us()/
     * delay_ms() using the DWT cycle counter ... Members B and C both
     * import it" — this loop is the placeholder to replace with that
     * shared implementation once it exists; not calibrated against any
     * particular core clock. */
    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 4000U; j++) { /* spin */ }
    }
}

#endif /* GPS_TARGET_STM32 */
