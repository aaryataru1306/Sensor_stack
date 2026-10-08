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

/*============================================================================*
 *                   u-blox NEO-M8N UBX PROTOCOL & CONFIGURATION              *
 *============================================================================*/

GPS_Status_t NEO_M8N_SendUBX(GPS_Handle_t *handle, uint8_t msg_class, uint8_t msg_id,
                            const uint8_t *payload, uint16_t length)
{
    if (handle == NULL || (!handle->initialized)) {
        return GPS_ERROR_NOT_INITIALIZED;
    }
    if (payload == NULL && length > 0U) {
        return GPS_ERROR_INVALID_PARAM;
    }

    /* UBX Frame: Sync(2) + Class(1) + ID(1) + Length(2) + Payload(N) + Checksum(2) */
    uint8_t header[6];
    header[0] = 0xB5U; /* Sync 1 */
    header[1] = 0x62U; /* Sync 2 */
    header[2] = msg_class;
    header[3] = msg_id;
    header[4] = (uint8_t)(length & 0xFFU);
    header[5] = (uint8_t)((length >> 8U) & 0xFFU);

    /* Fletcher 8-bit Checksum (RFC 1145) calculated over Class, ID, Length, and Payload */
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    for (int i = 2; i < 6; i++) {
        ck_a = (uint8_t)(ck_a + header[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    for (uint16_t i = 0U; i < length; i++) {
        ck_a = (uint8_t)(ck_a + payload[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    uint8_t checksum[2] = { ck_a, ck_b };

    /* Transmit frame */
    if (GPS_BusWrite(handle->bus_context, header, 6U) != 0) {
        handle->fault_count++;
        handle->last_error = GPS_ERROR_BUS;
        return GPS_ERROR_BUS;
    }
    if (length > 0U && payload != NULL) {
        if (GPS_BusWrite(handle->bus_context, payload, length) != 0) {
            handle->fault_count++;
            handle->last_error = GPS_ERROR_BUS;
            return GPS_ERROR_BUS;
        }
    }
    if (GPS_BusWrite(handle->bus_context, checksum, 2U) != 0) {
        handle->fault_count++;
        handle->last_error = GPS_ERROR_BUS;
        return GPS_ERROR_BUS;
    }

    return GPS_OK;
}

GPS_Status_t NEO_M8N_SetBaudRate(GPS_Handle_t *handle, uint32_t baud_rate)
{
    /* UBX-CFG-PRT (Class 0x06, ID 0x00, Length 20 bytes for UART1) */
    uint8_t payload[20] = {0};
    payload[0]  = 1U;    /* portID: 1 = UART1 */
    payload[4]  = 0xD0U; /* 8N1 mode (8 data bits, no parity, 1 stop bit) */
    payload[5]  = 0x08U;
    payload[8]  = (uint8_t)(baud_rate & 0xFFU);
    payload[9]  = (uint8_t)((baud_rate >> 8U) & 0xFFU);
    payload[10] = (uint8_t)((baud_rate >> 16U) & 0xFFU);
    payload[11] = (uint8_t)((baud_rate >> 24U) & 0xFFU);
    payload[12] = 0x07U; /* inProtoMask: UBX + NMEA + RTCM */
    payload[14] = 0x03U; /* outProtoMask: UBX + NMEA */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x00U, payload, sizeof(payload));
}

GPS_Status_t NEO_M8N_SetUpdateRate(GPS_Handle_t *handle, uint16_t rate_ms)
{
    /* UBX-CFG-RATE (Class 0x06, ID 0x08, Length 6 bytes) */
    uint8_t payload[6] = {0};
    payload[0] = (uint8_t)(rate_ms & 0xFFU);
    payload[1] = (uint8_t)((rate_ms >> 8U) & 0xFFU);
    payload[2] = 1U; /* navRate = 1 fix cycle */
    payload[4] = 1U; /* timeRef = 1 (GPS time) */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x08U, payload, sizeof(payload));
}

GPS_Status_t NEO_M8N_SetDynamicModel(GPS_Handle_t *handle, GPS_DynamicModel_t model)
{
    /* UBX-CFG-NAV5 (Class 0x06, ID 0x24, Length 36 bytes) */
    uint8_t payload[36] = {0};
    payload[0] = 0x01U; /* mask: apply dynModel */
    payload[2] = (uint8_t)model;
    payload[3] = 3U;    /* fixMode: 3 = Auto 2D/3D */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x24U, payload, sizeof(payload));
}

GPS_Status_t NEO_M8N_SetMessageRate(GPS_Handle_t *handle, uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    /* UBX-CFG-MSG (Class 0x06, ID 0x01, Length 8 bytes) */
    uint8_t payload[8] = {0};
    payload[0] = msg_class;
    payload[1] = msg_id;
    payload[3] = rate; /* UART1 rate */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x01U, payload, sizeof(payload));
}

GPS_Status_t NEO_M8N_SaveConfig(GPS_Handle_t *handle)
{
    /* UBX-CFG-CFG (Class 0x06, ID 0x09, Length 13 bytes) */
    uint8_t payload[13] = {0};
    payload[4]  = 0x1FU; /* saveMask: save ioPort, msgConf, infMsg, navConf, rxmConf */
    payload[12] = 0x17U; /* deviceMask: devBBR=1, devFlash=1, devEEPROM=1, devSpiFlash=1 */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x09U, payload, sizeof(payload));
}

GPS_Status_t NEO_M8N_HardwareReset(GPS_Handle_t *handle, GPS_ResetType_t reset_type)
{
    /* UBX-CFG-RST (Class 0x06, ID 0x13, Length 4 bytes) */
    uint8_t payload[4] = {0};
    payload[0] = (uint8_t)(reset_type & 0xFFU);
    payload[1] = (uint8_t)((reset_type >> 8U) & 0xFFU);
    payload[2] = 0x00U; /* resetMode: 0 = Hardware reset (watchdog) immediately */

    return NEO_M8N_SendUBX(handle, 0x06U, 0x13U, payload, sizeof(payload));
}
