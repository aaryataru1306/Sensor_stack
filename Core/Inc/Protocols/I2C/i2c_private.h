/**
 ******************************************************************************
 * @file    i2c_private.h
 * @brief   STM32F411 I2C register map, bit definitions, and peripheral
 *          base addresses — protocol-layer internal.
 * @details
 *   Everything in this file is a translation of RM0383 §22 (I2C
 *   interface) into C symbols. Nothing here is part of the public API;
 *   include i2c_interface.h instead.
 *
 *   The register layout below is the same for I2C1, I2C2 and I2C3 —
 *   only the base address differs. The F411's I2C peripherals do *not*
 *   expose the newer I2C_TIMINGR register (that's the F0/F3/F7/L4
 *   family); they use the classic CCR/TRISE scheme, which is what this
 *   driver programs.
 *
 * @note  Field comments say what a field does; see the reference manual
 *        for exact bit positions and reset values.
 ******************************************************************************
 */

#ifndef I2C_PRIVATE_H
#define I2C_PRIVATE_H

#include <stdint.h>

/*============================================================================*
 *                              BASE ADDRESSES                                *
 *============================================================================*/

/** @brief Peripheral base addresses, from RM0383 Table 3 (memory map). */
#define I2C1_BASE_ADDR   0x40005400UL   /**< APB1 */
#define I2C2_BASE_ADDR   0x40005800UL   /**< APB1 */
#define I2C3_BASE_ADDR   0x40005C00UL   /**< APB1 */

/*============================================================================*
 *                              REGISTER LAYOUT                               *
 *============================================================================*/

/**
 * @brief Register layout of one I2C peripheral.
 *
 * @details
 *   Byte-for-byte identical for I2C1/2/3 — a single struct definition is
 *   enough, and i2c_program.c just casts the right base address to a
 *   pointer of this type.
 *
 *   All fields are volatile: they are memory-mapped hardware, and the
 *   compiler must not reorder or elide accesses to them.
 */
typedef struct {
    volatile uint32_t CR1;    /**< 0x00 — Control register 1: PE, START, STOP,
                                   ACK, etc. */
    volatile uint32_t CR2;    /**< 0x04 — Control register 2: FREQ (PCLK1 in
                                   MHz), ITERREN, ITEVTEN, etc. */
    volatile uint32_t OAR1;   /**< 0x08 — Own address 1 (slave). */
    volatile uint32_t OAR2;   /**< 0x0C — Own address 2 (slave, dual-address
                                   mode). */
    volatile uint32_t DR;     /**< 0x10 — Data register (byte to send / last
                                   byte received). */
    volatile uint32_t SR1;    /**< 0x14 — Status register 1: SB, ADDR, BTF,
                                   RXNE, TXE, AF, errors. */
    volatile uint32_t SR2;    /**< 0x18 — Status register 2: MSL, BUSY, TRA,
                                   DUALF, GENCALL. */
    volatile uint32_t CCR;    /**< 0x1C — Clock control: FS, DUTY, CCR
                                   (SCL timing). */
    volatile uint32_t TRISE;  /**< 0x20 — Rise time: TRISE (max SCL/SDA rise
                                   time in PCLK1 cycles + 1). */
    volatile uint32_t FLTR;   /**< 0x24 — Digital noise filter: ANOFF,
                                   DNF. */
} I2C_Regs_t;

/*============================================================================*
 *                          CR1 — CONTROL REGISTER 1                          *
 *============================================================================*/

#define I2C_CR1_PE          (1U << 0)    /**< Peripheral enable. */
#define I2C_CR1_SMBUS       (1U << 1)    /**< SMBus mode select. */
#define I2C_CR1_SMBTYPE     (1U << 3)    /**< SMBus type: 0 = device,
                                              1 = host. */
#define I2C_CR1_ENARP       (1U << 4)    /**< SMBus ARP enable. */
#define I2C_CR1_ENPEC       (1U << 5)    /**< SMBus PEC enable. */
#define I2C_CR1_ENGC        (1U << 6)    /**< General call enable. */
#define I2C_CR1_NOSTRETCH   (1U << 7)    /**< Clock stretching disable
                                              (slave only). */
#define I2C_CR1_START       (1U << 8)    /**< Generate START (master). */
#define I2C_CR1_STOP        (1U << 9)    /**< Generate STOP (master). */
#define I2C_CR1_ACK         (1U << 10)   /**< ACK enable (master and slave). */
#define I2C_CR1_POS         (1U << 11)   /**< ACK/PEC position, for the
                                              two-byte receive trick —
                                              not used by this driver. */
#define I2C_CR1_PEC         (1U << 12)   /**< PEC transfer. */
#define I2C_CR1_ALERT       (1U << 13)   /**< SMBus alert. */
#define I2C_CR1_SWRST       (1U << 15)   /**< Software reset. */

/*============================================================================*
 *                          CR2 — CONTROL REGISTER 2                          *
 *============================================================================*/

#define I2C_CR2_FREQ_MASK   0x3FU        /**< FREQ field, bits[5:0]. */
#define I2C_CR2_FREQ_SHIFT  0U           /**< FREQ field starts here. */
#define I2C_CR2_ITERREN     (1U << 8)    /**< Error interrupt enable. */
#define I2C_CR2_ITEVTEN     (1U << 9)    /**< Event interrupt enable. */
#define I2C_CR2_ITBUFEN     (1U << 10)   /**< Buffer interrupt enable. */
#define I2C_CR2_DMAEN       (1U << 11)   /**< DMA enable. */
#define I2C_CR2_LAST        (1U << 12)   /**< DMA last-transfer enable. */

/*============================================================================*
 *                          SR1 — STATUS REGISTER 1                           *
 *============================================================================*/

#define I2C_SR1_SB          (1U << 0)    /**< Start bit generated (master).
                                              Cleared by reading SR1 then
                                              writing DR. */
#define I2C_SR1_ADDR        (1U << 1)    /**< Address sent/matched. Cleared by
                                              reading SR1 then SR2. */
#define I2C_SR1_BTF         (1U << 2)    /**< Byte transfer finished: a data
                                              byte has been sent or received
                                              and the shift register is
                                              empty. */
#define I2C_SR1_RXNE        (1U << 6)    /**< DR is not empty (a byte has
                                              arrived). */
#define I2C_SR1_TXE         (1U << 7)    /**< DR is empty (ready for the next
                                              byte). */
#define I2C_SR1_AF          (1U << 10)   /**< Acknowledge failure: the slave
                                              NACKed us. */
#define I2C_SR1_OVR         (1U << 11)   /**< Overrun/underrun (slave mode
                                              mostly). */
#define I2C_SR1_ARLO        (1U << 9)    /**< Arbitration lost (multi-master). */
#define I2C_SR1_BERR        (1U << 8)    /**< Bus error: misplaced
                                              START/STOP. */
#define I2C_SR1_TIMEOUT     (1U << 14)   /**< SCL/SDA timeout (only when the
                                              timeout feature is enabled —
                                              off by default). */
#define I2C_SR1_PECERR      (1U << 12)   /**< PEC error. */

/*============================================================================*
 *                          SR2 — STATUS REGISTER 2                           *
 *============================================================================*/

#define I2C_SR2_MSL         (1U << 0)    /**< Master/slave: 1 = master. */
#define I2C_SR2_BUSY        (1U << 1)    /**< Bus busy. */
#define I2C_SR2_TRA         (1U << 2)    /**< Transmitter/receiver:
                                              1 = transmitter. */
#define I2C_SR2_GENCALL     (1U << 4)    /**< General-call address received. */
#define I2C_SR2_DUALF       (1U << 7)    /**< Dual-address flag. */

/*============================================================================*
 *                          CCR — CLOCK CONTROL                               *
 *============================================================================*/

#define I2C_CCR_FS          (1U << 15)   /**< Fast-mode select: 0 = standard,
                                              1 = fast. */
#define I2C_CCR_DUTY        (1U << 14)   /**< Fast-mode duty: 0 = 2:1,
                                              1 = 16:9. */
#define I2C_CCR_CCR_MASK    0x0FFFU      /**< SCL timing value, bits[11:0]. */

/*============================================================================*
 *                          TIMING CONSTANTS                                  *
 *============================================================================*/

/**
 * @brief TRISE values (per RM0383 §22.6.10).
 *
 *   TRISE = (max SCL/SDA rise time in ns / PCLK1 period in ns) + 1
 *
 * With the standard-mode spec limit of 1000 ns rise time and a 1/48 MHz
 * = 20.8 ns PCLK1 period, TRISE = 1000/20.8 + 1 ≈ 49. For fast mode the
 * spec limit is 300 ns, giving TRISE ≈ 15. The classic ST examples just
 * hard-code the two values below for a "typical" PCLK1 of a few tens of
 * MHz and accept the small approximation — this driver does the same,
 * and the comments make the approximation visible.
 */
#define I2C_TRISE_STANDARD  0x31U   /**< Standard mode, ~1000 ns max rise. */
#define I2C_TRISE_FAST      0x09U   /**< Fast mode, ~300 ns max rise. */

#endif /* I2C_PRIVATE_H */
