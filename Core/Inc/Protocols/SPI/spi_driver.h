/**
 ******************************************************************************
 * @file    spi_driver.h
 * @brief   Public API for the bare-metal SPI1 master protocol layer
 *          (STM32F411CEU6).
 * @details
 *   Register-level SPI1 master driver, written from scratch. Zero HAL_*
 *   calls, zero dependency on any higher layer.
 *
 *   This is the bus the ICM20948 and the BMP388 share. Because two
 *   sensors hang off the same SCLK/MISO/MOSI lines, the chip-select pin
 *   is **not** owned by this driver — it is passed in on every
 *   transaction. That is the one meaningful difference between this
 *   driver and a single-sensor SPI bus driver, and it's what lets the
 *   two sensor drivers coexist without either of them being able to
 *   accidentally assert the other's CS.
 *
 *   The driver is stateless — there is no static or global mutable
 *   state anywhere in the .c. All peripheral state lives in the SPI1
 *   registers themselves, and all chip-select state lives in the GPIOA
 *   ODR/BSRR registers.
 *
 *   Timing: at 100 MHz SYSCLK and APB2 divided by 1, APB2 = 100 MHz.
 *   With BR = DIV16 the SCLK line runs at 6.25 MHz, which is under both
 *   the ICM20948's 7 MHz limit and the BMP388's 10 MHz limit. SPI mode
 *   is 0 (CPOL = 0, CPHA = 0), which both parts accept.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                          PIN ASSIGNMENTS                                   *
 *============================================================================*/

/**
 * @name SPI1 pin map (fixed by the STM32F411 AF5 mapping)
 *
 * The SCK / MISO / MOSI pins are decided by the silicon — AF5 on GPIOA
 * is the only pin set that exposes SPI1 on this part without a
 * remap. They are documented here for readability, not because the
 * caller needs them: SPI1_Init() configures them itself.
 * @{
 */
#define SPI1_SCK_PIN    5U   /**< PA5 — SPI1_SCK, AF5. */
#define SPI1_MISO_PIN   6U   /**< PA6 — SPI1_MISO, AF5. */
#define SPI1_MOSI_PIN   7U   /**< PA7 — SPI1_MOSI, AF5. */
/** @} */

/**
 * @name Chip-select pins (board-specific, passed per transaction)
 *
 * Both sensors share the SPI1 clock and data lines, so each needs its
 * own CS. These are just the *default* pin numbers this project wires
 * them to — the actual chip-select pin is an argument to every
 * transaction call, so a caller is free to use any GPIOA pin.
 *
 * @note ICM20948 CS is configured on PA1 (avoiding collision with
 *       USART2 on PA2/PA3).
 * @{
 */
#define SPI1_CS_ICM42688_PIN   1U   /**< PA1 — ICM42688-P chip-select, active low. */
#define SPI1_CS_ICM20948_PIN   1U   /**< PA1 — ICM20948 chip-select, active low. */
#define SPI1_CS_BMP388_PIN     4U   /**< PA4 — BMP388 chip-select, active low. */
/** @} */

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Enable clocks, configure GPIO, and bring SPI1 up as a master.
 *
 * @details
 *   Sequence:
 *     1. Enable the RCC clock for GPIOA and for SPI1.
 *     2. Configure PA5/PA6/PA7 as alternate-function push-pull, AF5,
 *        high speed.
 *     3. Configure PA1 and PA4 (the default CS pins) as plain
 *        push-pull outputs, and drive them **high** — both parts must
 *        see an idle-high CS before their first transaction.
 *     4. Program CR1: master, software NSS, BR = DIV16, SPI mode 0
 *        (CPOL = 0, CPHA = 0), 8-bit data, MSB-first.
 *     5. Set SPE=1 to enable the peripheral.
 *
 *   Idempotent — safe to call more than once. Callers should call this
 *   exactly once, from main.c, before bringing up any sensor driver
 *   that uses SPI1.
 *
 * @note  This function configures the CS pins listed above as part of
 *        the bring-up. If you wire a sensor to a different GPIOA pin,
 *        either move the define above or configure that pin yourself
 *        before the first transaction.
 */
void SPI1_Init(void);

/**
 * @brief Assert one chip-select line (drive it low).
 *
 * @details
 *   Directly writes the GPIOA BSRR register — one bus cycle, no
 *   read-modify-write, so an interrupt between two CS operations on
 *   different pins cannot corrupt either of them.
 *
 * @param[in] cs_pin  GPIOA pin number (0..15) to drive low.
 */
void SPI1_CS_Enable(uint8_t cs_pin);

/**
 * @brief Deassert one chip-select line (drive it high).
 *
 * @param[in] cs_pin  GPIOA pin number (0..15) to drive high.
 */
void SPI1_CS_Disable(uint8_t cs_pin);

/**
 * @brief Full-duplex transfer of a single byte.
 *
 * @details
 *   Blocks until the TX FIFO accepts the byte, blocks until the RX
 *   FIFO has one, and blocks until the peripheral is no longer busy
 *   before returning. The return value is whatever the slave shifted
 *   out while we shifted in — for a write-only transfer it is a
 *   throwaway byte, for a read it is the data.
 *
 * @param[in] data  Byte to transmit.
 *
 * @return The byte received during the same clock burst.
 *
 * @note  Exposed publicly only for unusual cases (bit-banged protocols
 *        on top of SPI, debug). Ordinary drivers should use
 *        SPI1_ReadRegisters() / SPI1_WriteRegister() instead.
 */
uint8_t SPI1_TransferByte(uint8_t data);

/**
 * @brief Transfer a buffer out and discard the received bytes.
 *
 * @param[in] data    Bytes to transmit. May be NULL only if length == 0.
 * @param[in] length  Number of bytes.
 *
 * @note  No chip-select is asserted — this is a raw buffer transfer.
 *        Use SPI1_WriteRegister() for a proper CS-framed transaction.
 */
void SPI1_WriteBuffer(const uint8_t *data, uint16_t length);

/**
 * @brief Read length bytes by clocking out 0xFF and capturing the
 *        reply.
 *
 * @param[out] buffer  Destination for received bytes. May be NULL only
 *                     if length == 0.
 * @param[in]  length  Number of bytes.
 *
 * @note  Same CS caveat as SPI1_WriteBuffer().
 */
void SPI1_ReadBuffer(uint8_t *buffer, uint16_t length);

/**
 * @brief Read a contiguous block of registers from one SPI slave.
 *
 * @details
 *   Complete, chip-select-framed transaction:
 *     CS low → send reg_addr → read length bytes → CS high
 *
 *   The register-address byte is sent exactly as passed — callers that
 *   need the top bit set for a read (as on the ICM20948) are expected
 *   to OR it in themselves. The BMP388, by contrast, needs an extra
 *   dummy byte between the address and the data, and the BMP388 driver
 *   handles that in its own bus wrapper.
 *
 * @param[in]  cs_pin    GPIOA pin number of the target slave's CS line.
 * @param[in]  reg_addr  Register address byte to send first.
 * @param[out] buffer    Destination for received bytes. May be NULL
 *                       only if length == 0.
 * @param[in]  length    Number of bytes to read.
 */
void SPI1_ReadRegisters(uint8_t cs_pin,
                        uint8_t reg_addr,
                        uint8_t *buffer,
                        uint16_t length);

/**
 * @brief Write a single register on one SPI slave.
 *
 * @details
 *   Complete, chip-select-framed transaction:
 *     CS low → send reg_addr → send value → CS high
 *
 *   Same "no bit twiddling" rule as SPI1_ReadRegisters(): the caller
 *   supplies the raw address byte. The ICM20948 driver ANDs off bit 7
 *   for a write; the BMP388 just passes the address through.
 *
 * @param[in] cs_pin    GPIOA pin number of the target slave's CS line.
 * @param[in] reg_addr  Register address byte to send first.
 * @param[in] value     Byte to write to that register.
 */
void SPI1_WriteRegister(uint8_t cs_pin,
                        uint8_t reg_addr,
                        uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* SPI_DRIVER_H */
