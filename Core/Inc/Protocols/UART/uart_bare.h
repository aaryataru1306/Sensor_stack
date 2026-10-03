/**
 ******************************************************************************
 * @file    uart_bare.h
 * @brief   Public API for the bare-metal UART/USART protocol layer
 *          (STM32F411CEU6).
 * @details
 *   Register-level USART1 / USART2 / USART6 driver, written from scratch
 *   against RM0383 §19 ("Universal synchronous asynchronous receiver
 *   transmitter"). Zero HAL_* calls, zero dependency on any higher layer.
 *
 *   This is the bus the GPS module sits on (USART2 by convention). The
 *   GPS driver never touches a USART register itself — it calls
 *   UART_Bare_ReceiveByte() / UART_Bare_Transmit() and nothing else.
 *
 *   The driver is *handle-based*: a UART_Bare_Handle_t carries the
 *   peripheral base address, baud-rate config, and initialization flag,
 *   and is passed to every call. There is no static or global mutable
 *   state anywhere in the .c — two independent UARTs on the same chip
 *   can be driven concurrently by two different handle instances. This
 *   matters here because a debug console on USART1 and the GPS on
 *   USART2 are a very common pairing on this board.
 *
 *   The RX path is deliberately **non-blocking**. A GPS receiver
 *   streams NMEA bytes asynchronously relative to your main loop, so
 *   the natural primitive is "is a byte waiting? give it to me" — not
 *   "block until a byte arrives." Blocking on RX would stall the main
 *   loop for up to a full NMEA sentence period (100 ms at 1 Hz, longer
 *   if you happen to catch the gap between fixes), which is exactly
 *   what a flight-control-adjacent main loop must not do.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework.
 ******************************************************************************
 */

#ifndef UART_BARE_H
#define UART_BARE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              PUBLIC TYPES                                  *
 *============================================================================*/

/**
 * @brief Every status the driver can return.
 *
 * @details
 *   The split between `_NO_DATA` and the error codes matters: a caller
 *   polling for the next byte will see `_NO_DATA` on almost every call,
 *   and that must not be treated as a fault. Everything else is a real
 *   problem with the wire, the peripheral, or the caller's setup.
 */
typedef enum {
    UART_BARE_OK = 0,                   /**< Byte received / transmitted
                                             as requested. */
    UART_BARE_ERROR_NO_DATA,            /**< RX FIFO / data register is
                                             empty. **Not an error** —
                                             this is the normal result of
                                             a non-blocking RX poll. */
    UART_BARE_ERROR_OVERRUN,            /**< A byte arrived before the
                                             previous one was read; the
                                             older byte is lost. */
    UART_BARE_ERROR_FRAMING,            /**< Stop bit missing — usually a
                                             baud-rate mismatch or noise
                                             on the line. */
    UART_BARE_ERROR_NOISE,              /**< Noise detected on the line. */
    UART_BARE_ERROR_PARITY,             /**< Parity mismatch (only
                                             meaningful if parity is
                                             enabled). */
    UART_BARE_ERROR_NOT_INITIALIZED,    /**< Called before
                                             UART_Bare_Init(). */
    UART_BARE_ERROR_TIMEOUT             /**< A blocking transmit ran past
                                             its iteration budget — the
                                             wire is stuck, the peripheral
                                             is not clocked, or the
                                             timeout is too tight. */
} UART_Bare_Status_t;

/**
 * @brief Compile-time-plus-runtime config for one UART instance.
 *
 * @details
 *   The caller fills one of these in and passes it to
 *   UART_Bare_Init(). After init, the handle keeps a copy; the caller's
 *   struct can go away.
 *
 *   `timeout_iters` is a raw loop-iteration budget, not a real
 *   millisecond count. A blocking transmit polls the TXE flag in a
 *   tight loop, and at a given core clock that loop runs a
 *   predictable-but-unknown number of iterations per millisecond. The
 *   caller picks a number comfortably above the worst-case transmit
 *   time at their baud rate and leaves it — the point is to break out
 *   of a stuck-wire hang, not to do precise timing.
 */
typedef struct {
    uint32_t peripheral_clock_hz;   /**< The APB clock actually feeding
                                         this USART [Hz]. Differs per
                                         peripheral (USART1 and USART6
                                         are on APB2, USART2 on APB1) and
                                         changes if the clock tree is
                                         reconfigured — pass the real
                                         value, not a guess. */
    uint32_t baud_rate;              /**< Target baud rate [bit/s]. 9600 is
                                         the near-universal power-on
                                         default for GPS modules; higher
                                         rates (38400, 57600, 115200)
                                         reduce sentence latency if the
                                         module supports them. */
    uint32_t timeout_iters;          /**< Busy-loop iteration budget for
                                         blocking operations. Used only
                                         by UART_Bare_Transmit(). */
} UART_Bare_Config_t;

/**
 * @brief One UART instance's full state.
 *
 * @details
 *   The base pointer is a `volatile uint32_t *` rather than a typed
 *   `USART_Regs_t *` because this header has no business knowing the
 *   register layout — that stays private to uart_bare.c. From the
 *   caller's point of view, this field is opaque.
 *
 *   Everything else is either a copy of the config the caller passed
 *   in, or a flag saying the instance is ready to use.
 */
typedef struct {
    volatile uint32_t *base;             /**< Opaque: the peripheral's
                                              base address. Set by
                                              UART_Bare_Init(). */
    UART_Bare_Config_t cfg;              /**< Copy of the caller's config. */
    uint8_t            initialized;      /**< Nonzero once Init() has
                                              succeeded. */
} UART_Bare_Handle_t;

/*============================================================================*
 *                              PUBLIC API                                    *
 *============================================================================*/

/**
 * @brief Bring one USART peripheral up: program the baud rate and
 *        enable TX, RX, and the peripheral itself.
 *
 * @details
 *   Sequence:
 *     1. Store the base address and config into the handle.
 *     2. Compute and write the BRR register from
 *        `peripheral_clock_hz` and `baud_rate`.
 *     3. Program CR1: UE (enable), TE (transmitter), RE (receiver),
 *        8N1 framing.
 *     4. Set handle->initialized = 1.
 *
 *   **RCC clock enable and GPIO pin configuration are NOT done here.**
 *   That is deliberate: which GPIO pins a given USART comes out on is a
 *   board decision, and it is the same class of decision as "which pin
 *   is chip-select" — it belongs in the platform bus file
 *   (gps_bus_stm32.c), not in the peripheral driver. See that file's
 *   `gps_bus_stm32_setup()` for the pattern.
 *
 *   Idempotent — calling it twice on the same handle reprograms the
 *   peripheral to the same state and is safe.
 *
 * @param[in,out] handle  Handle to initialize. Must not be NULL.
 * @param[in]     base    Peripheral base address as `volatile uint32_t *`.
 *                        For example, for USART2 on the STM32F411:
 *                        `(volatile uint32_t *)0x40004400UL`.
 *                        The caller is responsible for passing the
 *                        correct address for the peripheral they mean.
 * @param[in]     cfg     Baud-rate and timeout config. Must not be NULL,
 *                        and `peripheral_clock_hz` and `baud_rate` must
 *                        both be nonzero.
 *
 * @retval UART_BARE_OK                   Peripheral is up and idle.
 * @retval UART_BARE_ERROR_NOT_INITIALIZED  handle, base, or cfg was NULL,
 *                                        or a config field was zero.
 */
UART_Bare_Status_t UART_Bare_Init(UART_Bare_Handle_t *handle,
                                  volatile uint32_t *base,
                                  const UART_Bare_Config_t *cfg);

/**
 * @brief Non-blocking single-byte receive.
 *
 * @details
 *   The one function the GPS driver's bus read calls. Checks the RXNE
 *   flag once and returns immediately either way — it never spins on
 *   the flag, so a caller that pumps this in a main loop or an RX
 *   interrupt handler can never be stalled by a quiet line.
 *
 *   Order of checks matters: overrun, framing, and noise are examined
 *   **before** reading DR, because reading DR clears all three flags.
 *   If any of them is set, the byte in DR is unreliable and the caller
 *   gets a hard error instead.
 *
 * @param[in]  handle    Initialized UART handle.
 * @param[out] out       Receives the byte only if the return is
 *                       UART_BARE_OK. Untouched on any other return.
 *
 * @retval UART_BARE_OK                     A byte was waiting and has
 *                                          been written to *out.
 * @retval UART_BARE_ERROR_NO_DATA          No byte was waiting. Normal,
 *                                          expected, not an error.
 * @retval UART_BARE_ERROR_OVERRUN          A byte was lost — the previous
 *                                          one wasn't read in time.
 * @retval UART_BARE_ERROR_FRAMING          Stop-bit error — usually a
 *                                          baud mismatch.
 * @retval UART_BARE_ERROR_NOISE            Noise on the line.
 * @retval UART_BARE_ERROR_NOT_INITIALIZED  Called before Init().
 */
UART_Bare_Status_t UART_Bare_ReceiveByte(UART_Bare_Handle_t *handle,
                                         uint8_t *out);

/**
 * @brief Blocking transmit of `length` bytes.
 *
 * @details
 *   Polls TXE for each byte, then polls TC once at the end to make sure
 *   the last byte has fully shifted out before returning. Both polls
 *   are bounded by `cfg.timeout_iters`; a wire held low forever (or a
 *   peripheral that was never clocked) returns
 *   UART_BARE_ERROR_TIMEOUT rather than hanging the caller.
 *
 *   This is the correct primitive for sending an OSD command or an
 *   occasional module configuration string. It is **not** the right way
 *   to send a continuous data stream — for that, a ring-buffered TX
 *   path driven by the TXE interrupt would be the right design, and is
 *   out of scope here.
 *
 * @param[in] handle  Initialized UART handle.
 * @param[in] data    Bytes to send. May be NULL only if `length` == 0.
 * @param[in] length  Number of bytes to send.
 *
 * @retval UART_BARE_OK                     All bytes shifted out.
 * @retval UART_BARE_ERROR_NOT_INITIALIZED  Called before Init(), or a
 *                                          NULL data pointer with
 *                                          nonzero length.
 * @retval UART_BARE_ERROR_TIMEOUT          A flag never asserted within
 *                                          the iteration budget.
 */
UART_Bare_Status_t UART_Bare_Transmit(UART_Bare_Handle_t *handle,
                                      const uint8_t *data,
                                      uint16_t length);

/**
 * @brief Compute the value to write into the BRR register.
 *
 * @details
 *   The STM32 USART's baud-rate generator wants:
 *
 *     USARTDIV = PCLK / (16 * baud)
 *     BRR      = round(USARTDIV * 16)
 *
 *   which algebraically reduces to `round(PCLK / baud)` — the division
 *   below is that reduction, with a half-baud add so the result rounds
 *   to nearest instead of truncating.
 *
 *   Exposed publicly for one reason: a caller that only needs to change
 *   the baud rate at runtime (a GPS module that switched from 9600 to
 *   38400 mid-session, say) can compute the new BRR value with this and
 *   write it directly, without tearing down and rebuilding the handle.
 *   Everybody else can ignore it — UART_Bare_Init() calls it for them.
 *
 * @param[in] pclk  Peripheral clock feeding this USART [Hz].
 * @param[in] baud  Target baud rate [bit/s].
 *
 * @return The 16-bit BRR value. Caller is responsible for the register
 *         write.
 */
uint32_t UART_Bare_ComputeBRR(uint32_t pclk, uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* UART_BARE_H */
