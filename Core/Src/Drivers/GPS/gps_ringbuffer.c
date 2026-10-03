/**
 * @file    gps_ringbuffer.c
 * @brief   Implementation of the byte ring buffer declared in
 *          gps_ringbuffer.h. See that file for the design rationale.
 */

#include "gps_ringbuffer.h"

#define RB_MASK (GPS_RINGBUFFER_CAPACITY - 1U)

void GPS_RingBuffer_Init(GPS_RingBuffer_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->head = 0U;
    rb->tail = 0U;
    rb->dropped_bytes = 0U;
    for (uint16_t i = 0; i < GPS_RINGBUFFER_CAPACITY; i++) {
        rb->data[i] = 0U;
    }
}

uint16_t GPS_RingBuffer_Count(const GPS_RingBuffer_t *rb)
{
    if (rb == NULL) {
        return 0U;
    }
    return (uint16_t)((rb->head - rb->tail) & RB_MASK);
}

bool GPS_RingBuffer_IsEmpty(const GPS_RingBuffer_t *rb)
{
    return GPS_RingBuffer_Count(rb) == 0U;
}

bool GPS_RingBuffer_Push(GPS_RingBuffer_t *rb, uint8_t byte)
{
    if (rb == NULL) {
        return false;
    }

    uint16_t next_head = (uint16_t)((rb->head + 1U) & RB_MASK);
    if (next_head == rb->tail) {
        /* Full — one slot is always kept empty to distinguish full from
         * empty without a separate counter. Drop and count, never
         * overwrite (see header comment). */
        rb->dropped_bytes++;
        return false;
    }

    rb->data[rb->head] = byte;
    rb->head = next_head;
    return true;
}

bool GPS_RingBuffer_Pop(GPS_RingBuffer_t *rb, uint8_t *byte_out)
{
    if (rb == NULL || byte_out == NULL) {
        return false;
    }
    if (rb->head == rb->tail) {
        return false; /* empty */
    }

    *byte_out = rb->data[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) & RB_MASK);
    return true;
}
