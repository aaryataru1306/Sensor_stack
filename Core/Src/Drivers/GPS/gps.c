/**
 * @file    gps.c
 * @brief   Core implementation of the GPS/GNSS driver declared in gps.h.
 * @details Pipeline: GPS_BusReadByte() (gps_bus.h) -> GPS_RingBuffer_t
 *          (gps_ringbuffer.h) -> NMEA_Feed() (gps_nmea.h) ->
 *          NMEA_ParseSentence() -> handle->last_fix.
 *
 *          handle->rx_ring and handle->nmea point at caller-owned storage
 *          (no dynamic allocation anywhere in this driver — see gps.h's
 *          GPS_Handle_t field comments and this file's GPS_Init()). A
 *          typical caller does:
 *          @code
 *          static GPS_RingBuffer_t g_gps_ring;
 *          static NMEA_Parser_t    g_gps_parser;
 *          GPS_Handle_t gps = {0};
 *          gps.bus_context = ...;
 *          gps.rx_ring = (struct GPS_RingBuffer *)&g_gps_ring;
 *          gps.nmea    = (struct NMEA_Parser *)&g_gps_parser;
 *          GPS_Init(&gps);
 *          @endcode
 */

#include "gps.h"
#include "gps_bus.h"
#include "gps_nmea.h"
#include "gps_ringbuffer.h"
#include <string.h>

static GPS_RingBuffer_t *ring(GPS_Handle_t *handle)
{
    return (GPS_RingBuffer_t *)handle->rx_ring;
}

static NMEA_Parser_t *parser(GPS_Handle_t *handle)
{
    return (NMEA_Parser_t *)handle->nmea;
}

GPS_Status_t GPS_Init(GPS_Handle_t *handle)
{
    if (handle == NULL || handle->rx_ring == NULL || handle->nmea == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }

    GPS_RingBuffer_Init(ring(handle));
    NMEA_Parser_Init(parser(handle));
    memset(&handle->last_fix, 0, sizeof(handle->last_fix));

    handle->fault_count = 0U;
    handle->last_error  = GPS_OK;
    handle->initialized = true;
    return GPS_OK;
}

GPS_Status_t GPS_DeInit(GPS_Handle_t *handle)
{
    if (handle == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }
    handle->initialized = false;
    return GPS_OK;
}

GPS_Status_t GPS_Reset(GPS_Handle_t *handle)
{
    if (handle == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }
    if (!handle->initialized) {
        return GPS_ERROR_NOT_INITIALIZED;
    }

    GPS_RingBuffer_Init(ring(handle));
    NMEA_Parser_Init(parser(handle));
    memset(&handle->last_fix, 0, sizeof(handle->last_fix));
    return GPS_OK;
}

GPS_Status_t GPS_ReadDeviceID(GPS_Handle_t *handle, char id_out[2])
{
    if (handle == NULL || id_out == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }
    if (!handle->initialized) {
        return GPS_ERROR_NOT_INITIALIZED;
    }
    if ((handle->last_fix.sentences_parsed + handle->last_fix.unsupported_sentences) == 0U) {
        return GPS_ERROR_NOT_READY; /* no sentence seen yet (of any kind)
                                          -> talker unknown */
    }
    id_out[0] = handle->last_fix.talker_id[0];
    id_out[1] = handle->last_fix.talker_id[1];
    return GPS_OK;
}

GPS_Status_t GPS_Process(GPS_Handle_t *handle)
{
    if (handle == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }
    if (!handle->initialized) {
        return GPS_ERROR_NOT_INITIALIZED;
    }

    /* Stage 1: drain everything currently waiting on the bus into the
     * ring buffer. Non-blocking — GPS_BusReadByte() returning "no byte
     * waiting" (1) ends the drain for this call; it is not a fault. */
    for (;;) {
        uint8_t byte;
        int rc = GPS_BusReadByte(handle->bus_context, &byte);
        if (rc == 0) {
            (void)GPS_RingBuffer_Push(ring(handle), byte); /* drop-on-full
                                                                 is already
                                                                 counted by
                                                                 the ring
                                                                 buffer
                                                                 itself */
            continue;
        }
        if (rc == 1) {
            break; /* caught up */
        }
        handle->fault_count++;
        handle->last_error = GPS_ERROR_BUS;
        return GPS_ERROR_BUS;
    }

    /* Stage 2: feed every buffered byte through the NMEA framer, applying
     * each complete, checksum-valid sentence to last_fix as it completes. */
    uint8_t byte;
    while (GPS_RingBuffer_Pop(ring(handle), &byte)) {
        NMEA_FeedResult_t fr = NMEA_Feed(parser(handle), byte);
        switch (fr) {
            case NMEA_FEED_SENTENCE_READY:
                (void)NMEA_ParseSentence(parser(handle)->payload, &handle->last_fix);
                break;
            case NMEA_FEED_CHECKSUM_ERROR:
            case NMEA_FEED_OVERFLOW:
                handle->last_fix.checksum_errors++;
                break;
            case NMEA_FEED_PENDING:
            default:
                break;
        }
    }

    handle->last_error = GPS_OK;
    return GPS_OK;
}

GPS_Status_t GPS_ReadData(GPS_Handle_t *handle, GPS_Data_t *data)
{
    if (handle == NULL || data == NULL) {
        return GPS_ERROR_INVALID_PARAM;
    }
    if (!handle->initialized) {
        return GPS_ERROR_NOT_INITIALIZED;
    }

    GPS_Status_t st = GPS_Process(handle);
    if (st != GPS_OK) {
        return st;
    }

    if (handle->last_fix.sentences_parsed == 0U) {
        return GPS_ERROR_NOT_READY;
    }

    *data = handle->last_fix;
    return GPS_OK;
}

GPS_Status_t GPS_SendRawCommand(GPS_Handle_t *handle, const uint8_t *data, uint16_t length)
{
    if (handle == NULL || (data == NULL && length > 0U)) {
        return GPS_ERROR_INVALID_PARAM;
    }
    if (!handle->initialized) {
        return GPS_ERROR_NOT_INITIALIZED;
    }
    if (GPS_BusWrite(handle->bus_context, data, length) != 0) {
        handle->fault_count++;
        handle->last_error = GPS_ERROR_BUS;
        return GPS_ERROR_BUS;
    }
    return GPS_OK;
}
