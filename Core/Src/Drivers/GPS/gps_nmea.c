/**
 * @file    gps_nmea.c
 * @brief   Implementation of the NMEA0183 framer and field parser
 *          declared in gps_nmea.h. See that file for the two-layer
 *          design.
 */

#include "gps_nmea.h"
#include <string.h>

/*============================================================================*
 *                    LAYER 1: SENTENCE FRAMER IMPLEMENTATION                 *
 *============================================================================*/

void NMEA_Parser_Init(NMEA_Parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

static uint8_t hex_nibble(uint8_t c)
{
    if (c >= (uint8_t)'0' && c <= (uint8_t)'9') {
        return (uint8_t)(c - (uint8_t)'0');
    }
    if (c >= (uint8_t)'A' && c <= (uint8_t)'F') {
        return (uint8_t)(c - (uint8_t)'A' + 10U);
    }
    if (c >= (uint8_t)'a' && c <= (uint8_t)'f') {
        return (uint8_t)(c - (uint8_t)'a' + 10U);
    }
    return 0xFFU; /* not a hex digit */
}

NMEA_FeedResult_t NMEA_Feed(NMEA_Parser_t *parser, uint8_t byte)
{
    if (parser == NULL) {
        return NMEA_FEED_PENDING;
    }

    /* '$' always (re)starts a sentence, mid-frame or not — see the
     * function's header comment for why resyncing here is correct. */
    if (byte == (uint8_t)'$') {
        parser->payload_len = 0U;
        parser->in_sentence = true;
        parser->in_checksum = false;
        parser->checksum_digits_read = 0U;
        parser->running_checksum = 0U;
        parser->received_checksum = 0U;
        parser->overflowed = false;
        return NMEA_FEED_PENDING;
    }

    if (!parser->in_sentence) {
        return NMEA_FEED_PENDING; /* waiting for the next '$' */
    }

    if (byte == (uint8_t)'\r' || byte == (uint8_t)'\n') {
        if (!parser->in_sentence) {
            return NMEA_FEED_PENDING;
        }
        parser->in_sentence = false;

        if (parser->overflowed) {
            return NMEA_FEED_OVERFLOW;
        }
        /* A well-formed sentence always carries a 2-digit checksum in
         * this driver's accepted input; treat anything else (no '*' at
         * all, or fewer than 2 hex digits before the line ended) as a
         * checksum failure rather than silently accepting unverified
         * data. */
        if (!parser->in_checksum || parser->checksum_digits_read != 2U) {
            return NMEA_FEED_CHECKSUM_ERROR;
        }

        parser->payload[parser->payload_len] = '\0';
        if (parser->received_checksum != parser->running_checksum) {
            return NMEA_FEED_CHECKSUM_ERROR;
        }
        return NMEA_FEED_SENTENCE_READY;
    }

    if (parser->overflowed) {
        return NMEA_FEED_PENDING; /* discard until next '$' or terminator */
    }

    if (byte == (uint8_t)'*' && !parser->in_checksum) {
        parser->in_checksum = true;
        parser->checksum_digits_read = 0U;
        parser->received_checksum = 0U;
        return NMEA_FEED_PENDING; /* '*' itself is not part of the checksum
                                      and not stored in payload */
    }

    if (parser->in_checksum) {
        uint8_t nibble = hex_nibble(byte);
        if (nibble == 0xFFU || parser->checksum_digits_read >= 2U) {
            /* Not a valid hex digit where one was expected, or a third
             * digit turned up — this sentence cannot be valid. Keep
             * consuming bytes (don't bail out of in_sentence early) so a
             * stray non-hex byte doesn't desync framing of the CR/LF
             * that's still coming; the CR/LF handler above will reject
             * it via checksum_digits_read != 2. */
            parser->overflowed = true;
            return NMEA_FEED_PENDING;
        }
        parser->received_checksum = (uint8_t)((parser->received_checksum << 4) | nibble);
        parser->checksum_digits_read++;
        return NMEA_FEED_PENDING;
    }

    /* Ordinary payload byte, before '*': part of both the checksum and
     * the stored payload string. */
    parser->running_checksum = (uint8_t)(parser->running_checksum ^ byte);

    if (parser->payload_len >= NMEA_MAX_PAYLOAD_LEN) {
        parser->overflowed = true;
        return NMEA_FEED_PENDING;
    }
    parser->payload[parser->payload_len] = (char)byte;
    parser->payload_len++;
    return NMEA_FEED_PENDING;
}

/*============================================================================*
 *                    SHARED FIELD-SPLITTING UTILITY                          *
 *============================================================================*/

/**
 * @brief Copies the next comma-delimited field from *cursor into out
 *        (bounded, null-terminated), then advances *cursor past the
 *        comma. Written from scratch instead of strtok() so the caller's
 *        buffer is never mutated (the framer's payload can be parsed
 *        more than once, e.g. by a test) and so there is no hidden
 *        global/thread-local state (strtok's classic footgun) — matching
 *        DRIVER_STANDARD.md's "provide utility functions (e.g. simple
 *        buffer/string parsing)" instruction to write this ourselves.
 * @return true if a field (possibly empty) was extracted; false only
 *         when *cursor already points at the string's terminating NUL
 *         (i.e. there is no field left, not even an empty trailing one).
 */
static bool nmea_next_field(const char **cursor, char *out, size_t out_size)
{
    if (cursor == NULL || *cursor == NULL || **cursor == '\0') {
        if (out != NULL && out_size > 0U) {
            out[0] = '\0';
        }
        return false;
    }

    const char *start = *cursor;
    const char *p = start;
    while (*p != '\0' && *p != ',') {
        p++;
    }

    size_t field_len = (size_t)(p - start);
    if (out != NULL && out_size > 0U) {
        size_t copy_len = (field_len < out_size - 1U) ? field_len : (out_size - 1U);
        memcpy(out, start, copy_len);
        out[copy_len] = '\0';
    }

    *cursor = (*p == ',') ? (p + 1) : p; /* land on the NUL, not past it,
                                             when this was the last field */
    return true;
}

static bool field_is_empty(const char *field)
{
    return field == NULL || field[0] == '\0';
}

/** @brief Hand-rolled unsigned-integer parse for a bounded ASCII digit
 *         run (no external library, per the task brief; also avoids
 *         atoi()'s undefined behavior on non-numeric input by simply
 *         stopping at the first non-digit instead). */
static uint32_t parse_uint(const char *field)
{
    uint32_t value = 0U;
    if (field == NULL) {
        return 0U;
    }
    while (*field >= '0' && *field <= '9') {
        value = (value * 10U) + (uint32_t)(*field - '0');
        field++;
    }
    return value;
}

/** @brief Hand-rolled decimal (fixed or floating notation) parse — NMEA
 *         numeric fields are always plain ASCII decimals like "4807.038"
 *         or "1" or "-0.9", never scientific notation, so this doesn't
 *         need strtod()'s full generality. */
static double parse_double(const char *field)
{
    if (field == NULL || field[0] == '\0') {
        return 0.0;
    }

    bool negative = false;
    const char *p = field;
    if (*p == '-') {
        negative = true;
        p++;
    } else if (*p == '+') {
        p++;
    }

    double int_part = 0.0;
    while (*p >= '0' && *p <= '9') {
        int_part = (int_part * 10.0) + (double)(*p - '0');
        p++;
    }

    double frac_part = 0.0;
    if (*p == '.') {
        p++;
        double scale = 0.1;
        while (*p >= '0' && *p <= '9') {
            frac_part += (double)(*p - '0') * scale;
            scale *= 0.1;
            p++;
        }
    }

    double value = int_part + frac_part;
    return negative ? -value : value;
}

/** @brief Converts an NMEA "ddmm.mmmm" / "dddmm.mmmm" coordinate field
 *         plus its hemisphere letter into signed decimal degrees.
 * @param degree_digits  2 for latitude, 3 for longitude (NMEA pads
 *                        longitude's degrees to 3 digits, latitude to 2
 *                        — the only structural difference between the
 *                        two fields).
 */
static double nmea_coord_to_decimal(const char *field, char hemisphere, int degree_digits)
{
    if (field_is_empty(field)) {
        return 0.0;
    }

    char degrees_buf[4] = {0};
    int i = 0;
    for (; i < degree_digits && field[i] >= '0' && field[i] <= '9'; i++) {
        degrees_buf[i] = field[i];
    }
    degrees_buf[i] = '\0';

    double degrees = parse_double(degrees_buf);
    double minutes = parse_double(&field[i]);
    double decimal = degrees + (minutes / 60.0);

    if (hemisphere == 'S' || hemisphere == 'W' || hemisphere == 's' || hemisphere == 'w') {
        decimal = -decimal;
    }
    return decimal;
}

/** @brief Splits an NMEA "hhmmss.sss" (or bare "hhmmss") time field. */
static void nmea_parse_time(const char *field, uint8_t *hour, uint8_t *minute, float *second)
{
    if (field_is_empty(field) || hour == NULL || minute == NULL || second == NULL) {
        return;
    }
    char hh[3] = { field[0], field[1], '\0' };
    char mm[3] = { field[2], field[3], '\0' };
    *hour = (uint8_t)parse_uint(hh);
    *minute = (uint8_t)parse_uint(mm);
    *second = (float)parse_double(&field[4]);
}

/** @brief Splits an NMEA RMC "ddmmyy" date field, expanding the 2-digit
 *         year to 20xx.
 * @note  NMEA0183's 2-digit year is a known, real limitation of the
 *        sentence format itself (not something this driver chose to
 *        simplify) — every RMC-emitting receiver in the field today
 *        still only reports "yy". Assuming 2000+yy is correct for every
 *        currently-manufactured GPS module and stays correct until 2100;
 *        it is not correct for a receiver replaying pre-2000 dates
 *        (there are none). If this driver is still in service in 2100,
 *        this is the line to revisit. */
static void nmea_parse_date(const char *field, uint8_t *day, uint8_t *month, uint16_t *year)
{
    if (field_is_empty(field) || day == NULL || month == NULL || year == NULL) {
        return;
    }
    char dd[3] = { field[0], field[1], '\0' };
    char mm[3] = { field[2], field[3], '\0' };
    char yy[3] = { field[4], field[5], '\0' };
    *day = (uint8_t)parse_uint(dd);
    *month = (uint8_t)parse_uint(mm);
    *year = (uint16_t)(2000U + parse_uint(yy));
}

/*============================================================================*
 *                    LAYER 2: SENTENCE FIELD PARSER                          *
 *============================================================================*/

NMEA_SentenceType_t NMEA_IdentifySentence(const char *payload)
{
    if (payload == NULL || strlen(payload) < 5U) {
        return NMEA_SENTENCE_UNRECOGNIZED;
    }
    /* payload[0..1] = talker ID (GP/GN/GL/GA/BD/QZ/...), payload[2..4] =
     * sentence type — this driver deliberately does not care which
     * talker ID precedes "GGA"/"RMC" (see this function's header
     * comment). */
    if (memcmp(&payload[2], "GGA", 3) == 0) {
        return NMEA_SENTENCE_GGA;
    }
    if (memcmp(&payload[2], "RMC", 3) == 0) {
        return NMEA_SENTENCE_RMC;
    }
    return NMEA_SENTENCE_UNRECOGNIZED;
}

bool NMEA_ExtractTalkerId(const char *payload, char talker_id_out[2])
{
    if (payload == NULL || talker_id_out == NULL || strlen(payload) < 5U) {
        return false;
    }
    talker_id_out[0] = payload[0];
    talker_id_out[1] = payload[1];
    return true;
}

NMEA_SentenceType_t NMEA_ParseSentence(const char *payload, GPS_Data_t *fix)
{
    if (payload == NULL || fix == NULL) {
        return NMEA_SENTENCE_UNRECOGNIZED;
    }

    char talker[2];
    if (NMEA_ExtractTalkerId(payload, talker)) {
        fix->talker_id[0] = talker[0];
        fix->talker_id[1] = talker[1];
        fix->talker_id[2] = '\0';
    }

    NMEA_SentenceType_t type = NMEA_IdentifySentence(payload);
    switch (type) {
        case NMEA_SENTENCE_GGA:
            if (NMEA_ParseGGA(payload, fix)) {
                fix->sentences_parsed++;
            }
            break;
        case NMEA_SENTENCE_RMC:
            if (NMEA_ParseRMC(payload, fix)) {
                fix->sentences_parsed++;
            }
            break;
        case NMEA_SENTENCE_UNRECOGNIZED:
        default:
            fix->unsupported_sentences++;
            break;
    }
    return type;
}

bool NMEA_ParseGGA(const char *payload, GPS_Data_t *fix)
{
    if (payload == NULL || fix == NULL) {
        return false;
    }

    const char *cursor = payload;
    char field[16];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* header, e.g. "GPGGA" */

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 1: time */
    uint8_t hour = fix->utc_hour, minute = fix->utc_minute;
    float second = fix->utc_second;
    if (!field_is_empty(field)) {
        nmea_parse_time(field, &hour, &minute, &second);
    }

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 2: lat */
    char lat_field[16];
    memcpy(lat_field, field, sizeof(lat_field));
    bool have_lat = !field_is_empty(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 3: N/S */
    char ns = field[0];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 4: lon */
    char lon_field[16];
    memcpy(lon_field, field, sizeof(lon_field));
    bool have_lon = !field_is_empty(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 5: E/W */
    char ew = field[0];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 6: fix quality */
    uint32_t fix_quality = parse_uint(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 7: satellites */
    uint32_t satellites = parse_uint(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 8: HDOP */
    double hdop = field_is_empty(field) ? fix->hdop : parse_double(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 9: altitude */
    double altitude = field_is_empty(field) ? fix->altitude_m : parse_double(field);

    /* Fields 10-13 (altitude unit, geoid separation + unit, DGPS age/ID)
     * are not surfaced in GPS_Data_t (no flight-control consumer named in
     * the task brief needs them) — intentionally not parsed. */

    fix->utc_hour = hour;
    fix->utc_minute = minute;
    fix->utc_second = second;
    if (have_lat) {
        fix->latitude_deg = nmea_coord_to_decimal(lat_field, ns, 2);
    }
    if (have_lon) {
        fix->longitude_deg = nmea_coord_to_decimal(lon_field, ew, 3);
    }
    fix->fix_quality = (GPS_FixQuality_t)fix_quality;
    fix->satellites_in_use = (uint8_t)satellites;
    fix->hdop = hdop;
    fix->altitude_m = altitude;

    return true;
}

bool NMEA_ParseRMC(const char *payload, GPS_Data_t *fix)
{
    if (payload == NULL || fix == NULL) {
        return false;
    }

    const char *cursor = payload;
    char field[16];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* header */

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 1: time */
    uint8_t hour = fix->utc_hour, minute = fix->utc_minute;
    float second = fix->utc_second;
    if (!field_is_empty(field)) {
        nmea_parse_time(field, &hour, &minute, &second);
    }

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 2: status A/V */
    bool status_valid = (field[0] == 'A');

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 3: lat */
    char lat_field[16];
    memcpy(lat_field, field, sizeof(lat_field));
    bool have_lat = !field_is_empty(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 4: N/S */
    char ns = field[0];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 5: lon */
    char lon_field[16];
    memcpy(lon_field, field, sizeof(lon_field));
    bool have_lon = !field_is_empty(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 6: E/W */
    char ew = field[0];

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 7: speed, knots */
    double speed_knots = field_is_empty(field) ? fix->speed_knots : parse_double(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 8: course */
    double course = field_is_empty(field) ? fix->course_deg : parse_double(field);

    if (!nmea_next_field(&cursor, field, sizeof(field))) return false; /* 9: date */
    uint8_t day = fix->utc_day, month = fix->utc_month;
    uint16_t year = fix->utc_year;
    if (!field_is_empty(field)) {
        nmea_parse_date(field, &day, &month, &year);
    }

    /* Fields 10-12 (magnetic variation + sign, and the optional NMEA 2.3+
     * "mode" field) are not surfaced in GPS_Data_t — not needed by any
     * consumer named in the task brief, and reading them without
     * validating the sentence's NMEA version would risk misparsing an
     * older receiver's shorter RMC. */

    fix->utc_hour = hour;
    fix->utc_minute = minute;
    fix->utc_second = second;
    fix->rmc_status_valid = status_valid;
    if (have_lat) {
        fix->latitude_deg = nmea_coord_to_decimal(lat_field, ns, 2);
    }
    if (have_lon) {
        fix->longitude_deg = nmea_coord_to_decimal(lon_field, ew, 3);
    }
    fix->speed_knots = speed_knots;
    fix->course_deg = course;
    fix->utc_day = day;
    fix->utc_month = month;
    fix->utc_year = year;

    return true;
}
