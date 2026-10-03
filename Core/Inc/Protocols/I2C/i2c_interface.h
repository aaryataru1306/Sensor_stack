/**
 ******************************************************************************
 * @file    i2c_interface.h
 * @brief   Public API for the bare-metal I2C master protocol layer
 *          (STM32F411CEU6).
 * @details
 *   Register-level I2C1 / I2C2 / I2C3 master driver, written from scratch
 *   against RM0383 §22 ("Inter-integrated circuit (I2C) interface").
 *   Zero HAL_* calls, zero dependency on any higher layer.
 *
 *   This is the bus the MPU6050 (and any future I2C sensor) sits on.
 *   Sensor drivers call I2C_enumMasterTransmit() / Receive() /
 *   TransmitReceive() and never touch a register themselves — the whole
 *   point of the split is that swapping the STM32 for a different MCU
 *   would only mean rewriting this file, not every sensor driver above it.
 *
 *   The driver is *handle-based*: an I2C_Config_t carries the channel,
 *   clock rate, and PCLK1 frequency, and is passed to every call. There
 *   is no static or global mutable state anywhere in the .c — two
 *   independent buses on the same chip can be driven concurrently by two
 *   different I2C_Config_t instances.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include "ErrTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              PUBLIC TYPES                                  *
 *============================================================================*/

/**
 * @brief Which I2C peripheral to drive.
 *
 * @note  All three peripherals exist on the STM32F411CEU6. The Black Pill
 *        board brings out at least one pin pair for each.
 */
typedef enum {
    I2C_CHANNEL1 = 0,   /**< I2C1 — usually PB6 (SCL) / PB7 (SDA). */
    I2C_CHANNEL2,       /**< I2C2 — usually PB10 (SCL) / PB3 (SDA). */
    I2C_CHANNEL3        /**< I2C3 — usually PA8 (SCL) / PB4 (SDA). */
} I2C_Channel_t;

/**
 * @brief Bus mode. Only I2C_MODE_I2C is implemented today; SMBus is
 *        reserved for a future device that actually needs it.
 */
typedef enum {
    I2C_MODE_I2C = 0,
    I2C_MODE_SMBUS
} I2C_Mode_t;

/**
 * @brief Whether to ACK incoming bytes during a master receive.
 *
 * @note  This is the *peripheral-wide* ACK setting — the master always
 *        NACKs the very last byte of a receive regardless of this value.
 *        Leave it at ENABLE unless you know why you need the other one.
 */
typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
} I2C_Ack_t;

/**
 * @brief Slave address width.
 *
 * @note  Every sensor in this project uses 7-bit addressing. 10-bit is
 *        declared for completeness — if you enable it, remember that a
 *        10-bit address is sent as two bytes on the wire, and the
 *        peripheral takes care of that internally.
 */
typedef enum {
    I2C_ADDR_MODE_7BIT = 0,
    I2C_ADDR_MODE_10BIT
} I2C_AddrMode_t;

/**
 * @brief Bus speed mode.
 *
 * @note  STANDARD is up to 100 kHz, FAST is up to 400 kHz. There is no
 *        FAST_PLUS (1 MHz) enum here — the STM32F411's I2C peripheral
 *        technically supports it, but every sensor in this project is
 *        rated for 400 kHz max, so the extra mode would be a footgun.
 */
typedef enum {
    I2C_SPEED_STANDARD = 0,
    I2C_SPEED_FAST
} I2C_Speed_t;

/**
 * @brief Fast-mode duty cycle (Tlow/Thigh ratio).
 *
 * @note  Only meaningful when Speed == I2C_SPEED_FAST. DUTY_2 gives
 *        Tlow/Thigh = 2 (the standard, works with everything); DUTY_16_9
 *        gives 16/9 (faster rise time on loaded buses, needs correct
 *        pull-ups to be safe). When in doubt, use DUTY_2.
 */
typedef enum {
    I2C_DUTY_2 = 0,
    I2C_DUTY_16_9
} I2C_DutyCycle_t;

/**
 * @brief One bus configuration instance.
 *
 * @details
 *   Everything the driver needs to program the peripheral. All fields
 *   must be set before calling I2C_enumInit(); after that the struct is
 *   read-only from the driver's point of view and can live in ROM
 *   (declare it `static const` if you want).
 *
 *   There is no opaque bus-context field here — the struct *is* the
 *   context. Sensor drivers hold a pointer to one and pass it to every
 *   transaction call.
 */
typedef struct {
    I2C_Channel_t    Channel;         /**< Which peripheral. */
    I2C_Mode_t       Mode;            /**< I2C vs SMBus. */
    I2C_Ack_t        ACK;             /**< Peripheral-wide ACK enable. */
    I2C_AddrMode_t   AddressMode;     /**< 7-bit vs 10-bit slave addresses. */
    uint16_t         OwnAddress;      /**< Our own slave address (unused in
                                           master-only mode; set to 0). */
    I2C_Speed_t      Speed;           /**< Standard vs Fast mode. */
    I2C_DutyCycle_t  FM_DutyCycle;    /**< Only used when Speed is FAST. */
    uint32_t         ClockSpeed_Hz;   /**< Target SCL clock [Hz] —
                                           100000 or 400000 typically. */
    uint32_t         PCLK1_Hz;        /**< Actual APB1 clock feeding the
                                           peripheral [Hz]. Must match the
                                           real clock tree, or the CCR/
                                           TRISE calculation mis-times the
                                           bus. */
} I2C_Config_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Enable clocks, configure GPIO pins, and bring the peripheral up.
 *
 * @details
 *   Sequence:
 *     1. Enable the RCC clock for the chosen I2C peripheral and for the
 *        GPIO port(s) it uses.
 *     2. Configure SCL and SDA as alternate-function open-drain with the
 *        correct AF number for that peripheral.
 *     3. Program CR2.FREQ with PCLK1/1e6 (the peripheral's internal
 *        reference for its timing generators).
 *     4. Program CCR (SCL timing) and TRISE (max rise time) from
 *        ClockSpeed_Hz and Speed/FM_DutyCycle.
 *     5. Program OAR1 with OwnAddress (only meaningful in slave mode,
 *        but harmless to set).
 *     6. Set PE=1 in CR1 to enable the peripheral.
 *
 *   Idempotent — calling it twice on the same channel is safe and simply
 *   reprograms the peripheral to the same state.
 *
 * @param[in,out] cfg  Configuration struct. Must not be NULL; all fields
 *                     must already be set by the caller.
 *
 * @retval OK             Peripheral is up and idle.
 * @retval NULL_POINTER   cfg was NULL.
 * @retval INVALID_PARAM  Channel is out of range, or ClockSpeed_Hz /
 *                        PCLK1_Hz is 0.
 * @retval TIMEOUT_STATE  The peripheral failed to come out of reset.
 */
ErrorState_t I2C_enumInit(I2C_Config_t *cfg);

/**
 * @brief Master transmit: START, address+W, N data bytes, STOP.
 *
 * @details
 *   The classic "write N bytes to a slave" transaction. Used, for
 *   example, to send a register address that a subsequent receive will
 *   read from, or to send a configuration byte with no reply expected.
 *
 * @param[in] cfg       Initialized bus configuration.
 * @param[in] addr7     7-bit slave address, right-aligned (no R/W bit).
 *                      For an MPU6050 with AD0 low, this is 0x68.
 * @param[in] data      Bytes to send. May be NULL only if len == 0.
 * @param[in] len       Number of bytes to send.
 *
 * @retval OK             All bytes acknowledged and STOP issued.
 * @retval NULL_POINTER   cfg NULL, or (data NULL and len > 0).
 * @retval INVALID_PARAM  len == 0 (a bare address-only transaction is
 *                        not a valid master transmit here — use the
 *                        TransmitReceive form with rxlen == 0 instead).
 * @retval NOK            Slave NACKed an address or data byte.
 * @retval TIMEOUT_STATE  A bus phase did not complete in time (SDA/SCL
 *                        held low, missing pull-ups, or the slave is
 *                        holding the bus).
 */
ErrorState_t I2C_enumMasterTransmit(I2C_Config_t *cfg,
                                    uint8_t addr7,
                                    const uint8_t *data,
                                    uint16_t len);

/**
 * @brief Master receive: START, address+R, N data bytes, STOP.
 *
 * @details
 *   No write phase — this is the correct primitive only for a slave
 *   that has a fixed current-register-pointer you don't need to set,
 *   or that streams on read. **Almost every register-mapped sensor in
 *   this project needs the TransmitReceive form instead** (write the
 *   register address, repeated START, read).
 *
 *   The last byte is NACKed internally before STOP, as required by the
 *   I2C spec — the caller doesn't need to arrange that.
 *
 * @param[in]  cfg    Initialized bus configuration.
 * @param[in]  addr7  7-bit slave address.
 * @param[out] data   Buffer for received bytes. May be NULL only if
 *                    len == 0.
 * @param[in]  len    Number of bytes to receive.
 *
 * @retval OK             All bytes received and STOP issued.
 * @retval NULL_POINTER   cfg NULL, or (data NULL and len > 0).
 * @retval INVALID_PARAM  len == 0.
 * @retval NOK            Slave NACKed the address byte.
 * @retval TIMEOUT_STATE  A bus phase did not complete in time.
 */
ErrorState_t I2C_enumMasterReceive(I2C_Config_t *cfg,
                                   uint8_t addr7,
                                   uint8_t *data,
                                   uint16_t len);

/**
 * @brief Combined master write-then-read with a repeated START.
 *
 * @details
 *   The workhorse of register-mapped I2C. On the wire:
 *
 *     START, addr+W, [tx bytes], repeated START, addr+R, [rx bytes], STOP
 *
 *   For the classic "read N bytes from register R" pattern:
 *     tx = { R }, txlen = 1, rx points to an N-byte buffer, rxlen = N.
 *
 *   Allowing txlen == 0 degenerates to a plain receive with a
 *   preceding START — useful for a slave whose register pointer is
 *   already correct. Allowing rxlen == 0 degenerates to a plain
 *   transmit — useful if you want one code path for both cases.
 *
 *   This is the one function almost every caller actually uses.
 *
 * @param[in]  cfg     Initialized bus configuration.
 * @param[in]  addr7   7-bit slave address.
 * @param[in]  tx      Bytes to write before the repeated START. May be
 *                     NULL only if txlen == 0.
 * @param[in]  txlen   Number of bytes to write.
 * @param[out] rx      Buffer for received bytes. May be NULL only if
 *                     rxlen == 0.
 * @param[in]  rxlen   Number of bytes to receive.
 *
 * @retval OK             Whole transaction completed and STOP issued.
 * @retval NULL_POINTER   cfg NULL, or a data pointer NULL with its
 *                        length > 0.
 * @retval INVALID_PARAM  Both txlen and rxlen are 0.
 * @retval NOK            Slave NACKed an address or data byte.
 * @retval TIMEOUT_STATE  A bus phase did not complete in time.
 */
ErrorState_t I2C_enumMasterTransmitReceive(I2C_Config_t *cfg,
                                           uint8_t addr7,
                                           const uint8_t *tx,
                                           uint16_t txlen,
                                           uint8_t *rx,
                                           uint16_t rxlen);

#ifdef __cplusplus
}
#endif

#endif /* I2C_INTERFACE_H */
