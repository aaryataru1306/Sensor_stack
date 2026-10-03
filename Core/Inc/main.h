/**
 ******************************************************************************
 * @file    main.h
 * @brief   Top-level project header: pin map, system functions, and the
 *          forward declarations main.c needs.
 * @details
 *   This is the single source of truth for how the STM32F411CEU6 Black
 *   Pill is wired in this project. Every pin assignment is here; the
 *   individual drivers never hardcode a GPIO number except through the
 *   defines below (or through their own `<driver>_config.h` if the pin
 *   is truly driver-private — for sensors that share a bus, the shared
 *   pin belongs here).
 *
 *   Deliberately does NOT include stm32f4xx_hal.h. This project is
 *   bare-metal: every peripheral is touched through the register-level
 *   protocol drivers in Core/Src/Protocols/, never through HAL. If a
 *   future file genuinely needs a HAL symbol, it can include the HAL
 *   header itself — the top-level header should not drag HAL into every
 *   translation unit.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                            PIN MAP — SPI1 BUS                              *
 *============================================================================*/

/**
 * @name SPI1 pin map
 *
 * The ICM20948 and the BMP388 share SPI1. SCK/MISO/MOSI are fixed by the
 * STM32F411's alternate-function mapping — AF5 on GPIOA is the only pin
 * set that exposes SPI1 without a remap. Chip-select is per-sensor and
 * driven manually as plain GPIO.
 *
 * @note  The CS pin *numbers* are also defined inside
 *        Protocols/SPI/spi_driver.h (SPI1_CS_ICM20948_PIN, etc.) since
 *        that's the layer that actually writes to them. The defines here
 *        are the project-level view — same numbers, documented
 *        together with the rest of the pinout. If you change one, change
 *        both.
 * @{
 */
#define PIN_SPI1_SCK        5U    /**< PA5 — SPI1_SCK, AF5. */
#define PIN_SPI1_MISO       6U    /**< PA6 — SPI1_MISO, AF5. */
#define PIN_SPI1_MOSI       7U    /**< PA7 — SPI1_MOSI, AF5. */

#define PIN_ICM20948_CS     3U    /**< PA3 — ICM20948 chip-select, active low. */
#define PIN_BMP388_CS       4U    /**< PA4 — BMP388 chip-select, active low. */

/**
 * @warning If you run the GPS on USART2 (PA2 = TX, PA3 = RX), PA3 is
 *          taken and **cannot** be the ICM20948 CS. In that case move
 *          the ICM CS to a free pin (PB0, PB1, PA1, ...) and update
 *          PIN_ICM20948_CS here and SPI1_CS_ICM20948_PIN in
 *          spi_driver.h.
 */
/** @} */

/*============================================================================*
 *                            PIN MAP — I2C1 BUS                              *
 *============================================================================*/

/**
 * @name I2C1 pin map
 *
 * The MPU6050 sits on I2C1. Two pin pairs are available on the F411 for
 * I2C1 with AF4: PB6/PB7 (the Black Pill default) and PB8/PB9. The
 * defines below assume PB6/PB7 — the physical pin configuration is done
 * inside Protocols/I2C/i2c_program.c, which reads these numbers when it
 * programs the GPIO alternate-function registers.
 * @{
 */
#define PIN_I2C1_SCL        6U    /**< PB6 — I2C1_SCL, AF4, open-drain. */
#define PIN_I2C1_SDA        7U    /**< PB7 — I2C1_SDA, AF4, open-drain. */
/** @} */

/*============================================================================*
 *                            PIN MAP — USART2 (GPS)                          *
 *============================================================================*/

/**
 * @name USART2 pin map
 *
 * The GPS module streams NMEA over USART2. PA2 (TX) and PA3 (RX) are
 * the reset-default mapping — no AF remap needed.
 *
 * @warning These pins **collide** with PIN_ICM20948_CS (PA3) above. If
 *          both the ICM20948 and the GPS are populated on the same
 *          board, one of them must move. Common resolutions:
 *            - Move ICM CS to PB0, keep GPS on PA2/PA3.
 *            - Move GPS to USART1 (PA9/PA10 or PB6/PB7), keep ICM on PA3.
 *          Neither is done here — pick one and edit the pin defines.
 * @{
 */
#define PIN_USART2_TX       2U    /**< PA2 — USART2_TX, AF7. */
#define PIN_USART2_RX       3U    /**< PA3 — USART2_RX, AF7. */
/** @} */

/*============================================================================*
 *                            PIN MAP — LED (optional)                         *
 *============================================================================*/

/**
 * @name Onboard LED
 *
 * The Black Pill's user LED is on PC13, active-low (drive low to light).
 * Not used by any driver, but worth having a named define for it so a
 * bring-up blinky is one line away.
 * @{
 */
#define PIN_LED             13U   /**< PC13 — user LED, active low. */
/** @} */

/*============================================================================*
 *                          PERIPHERAL BASE ADDRESSES                          *
 *============================================================================*/

/**
 * @name Peripheral base addresses
 *
 * Only the ones main.c actually needs — the bus drivers know their own
 * base addresses internally (in spi_driver.c, i2c_program.c, and
 * uart_bare.c respectively). The GPS driver's bus file needs the
 * USART2 address, so it's provided here for convenience.
 *
 * @note  These are **not** the complete set — they're the subset a
 *        top-level caller might reasonably need to pass to a bus
 *        setup function like gps_bus_stm32_setup().
 * @{
 */
#define PERIPH_USART2_BASE  0x40004400UL   /**< USART2, APB1. */
#define PERIPH_USART1_BASE  0x40011000UL   /**< USART1, APB2. */
#define PERIPH_USART6_BASE  0x40011400UL   /**< USART6, APB2. */
#define PERIPH_I2C1_BASE    0x40005400UL   /**< I2C1, APB1. */
#define PERIPH_SPI1_BASE    0x40013000UL   /**< SPI1, APB2. */
/** @} */

/*============================================================================*
 *                          SYSTEM CLOCK FREQUENCIES                          *
 *============================================================================*/

/**
 * @name Clock tree constants
 *
 * Match what SystemClock_Config() actually programs. The bus drivers
 * take these as **arguments** (not as compile-time constants) so a
 * clock change only requires editing the call sites, but these defines
 * are here so the call sites in main.c read consistently.
 *
 * @note  The values below assume the recommended HSE 25 MHz -> PLL
 *        (M=25, N=200, P=2) -> SYSCLK 100 MHz configuration:
 *          - HCLK  = 100 MHz (AHB /1)
 *          - PCLK1 =  50 MHz (APB1 /2)
 *          - PCLK2 = 100 MHz (APB2 /1)
 *        If you change the PLL settings, change these too — the I2C
 *        driver's CCR/TRISE math and the UART driver's BRR math both
 *        depend on them being right.
 * @{
 */
#define SYSCLK_HZ   100000000UL   /**< System clock after PLL. */
#define HCLK_HZ     100000000UL   /**< AHB clock. */
#define PCLK1_HZ     50000000UL   /**< APB1 clock (I2C1, USART2, TIM6). */
#define PCLK2_HZ    100000000UL   /**< APB2 clock (SPI1, USART1). */
/** @} */

/*============================================================================*
 *                          PUBLIC FUNCTION PROTOTYPES                        *
 *============================================================================*/

/**
 * @brief Configure the system clock (HSE + PLL) and the flash latency.
 *
 * @details
 *   Runs on every boot, before any peripheral is initialized. Programs
 *   the RCC PLL and switches SYSCLK to it. Must be called from main()
 *   before the bus bring-up functions — everything downstream assumes
 *   the frequencies in the SYSCLK_HZ / PCLK1_HZ / PCLK2_HZ defines are
 *   already true.
 */
void SystemClock_Config(void);

/**
 * @brief Fatal error handler: disables interrupts and hangs.
 *
 * @details
 *   Every init path in main.c funnels here on failure. Disabling IRQs
 *   before the infinite loop makes the failure visible to a debugger
 *   (the core is stopped mid-hang, stack intact) rather than allowing
 *   a fault to re-enter the handler in an unpredictable way.
 *
 * @note  Marked `__attribute__((noreturn))` in the .c so the compiler
 *        knows not to fall through after a call.
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
