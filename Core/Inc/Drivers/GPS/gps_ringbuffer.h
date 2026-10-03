/**
 * @file    gps_ringbuffer.h
 * @brief   Small single-producer/single-consumer byte ring buffer.
 * @details Task brief ("Provide utility functions e.g. simple
 *          buffer/string parsing") and Phase2_BareMetal_Task_Division.pdf
 *          Member C task 2 ("A small ring buffer, since GPS streams NMEA
 *          sentences continuously and asynchronously relative to your
 *          main loop") both call this out explicitly as Member C's own
 *          utility to write, not something borrowed from A/B's layers.
 *
 *          Deliberately NOT named `gps_registers.h` (DRIVER_STANDARD.md's
 *          usual second header) — GPS/NMEA has no register map to
 *          document; this file's job is the closest equivalent concern
 *          for a streaming, address-less UART protocol. See
 *          `Drivers/GPS/README.md` "Deviations from DRIVER_STANDARD.md"
 *          for the full rationale.
 *
 * @note    Fixed capacity, no dynamic allocation (bare-metal target).
 *          Capacity must be a power of two so index wraparound is a
 *          cheap mask instead of a modulo.
 */

#ifndef GPS_RINGBUFFER_H
#define GPS_RINGBUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Capacity in bytes. 128 comfortably holds the longest NMEA0183
 *         sentence (82 chars incl. '$'..CRLF, per the spec) plus headroom
 *         for a second sentence's worth of margin while the consumer is
 *         momentarily behind. Must stay a power of two (see file header). */
#define GPS_RINGBUFFER_CAPACITY 128U

typedef struct GPS_RingBuffer {
    uint8_t  data[GPS_RINGBUFFER_CAPACITY];
    uint16_t head; /**< Next write index. Producer-owned. */
    uint16_t tail; /**< Next read index. Consumer-owned. */
    uint32_t dropped_bytes; /**< Incremented instead of overwriting on a
                                  full buffer — silent data loss is exactly
                                  the kind of bug this driver standard's
                                  "every bus call's return value is
                                  checked" rule exists to prevent. */
} GPS_RingBuffer_t;

/** @brief Zeroes all state. Safe to call on an already-in-use buffer to
 *         discard its contents (used by GPS_Reset()). */
void GPS_RingBuffer_Init(GPS_RingBuffer_t *rb);

/**
 * @brief Pushes one byte.
 * @retval true if stored; false if the buffer was full (byte dropped,
 *         dropped_bytes incremented) — never silently overwrites the
 *         oldest unread byte, which would corrupt whichever NMEA sentence
 *         is currently being reassembled.
 */
bool GPS_RingBuffer_Push(GPS_RingBuffer_t *rb, uint8_t byte);

/**
 * @brief Pops one byte.
 * @retval true if a byte was available and written to *byte_out; false if
 *         the buffer was empty (*byte_out left untouched).
 */
bool GPS_RingBuffer_Pop(GPS_RingBuffer_t *rb, uint8_t *byte_out);

/** @brief Number of unread bytes currently buffered. */
uint16_t GPS_RingBuffer_Count(const GPS_RingBuffer_t *rb);

/** @brief True if Count() == 0. */
bool GPS_RingBuffer_IsEmpty(const GPS_RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* GPS_RINGBUFFER_H */
