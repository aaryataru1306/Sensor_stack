/**
 * @file    gps_nmea.h
 * @brief   NMEA0183 byte-level sentence framer and field parser, written
 *          from scratch (Phase2_BareMetal_Task_Division.pdf Member C
 *          task 3: "NMEA sentence parser, written from scratch (no
 *          external library)").
 * @details Two independent layers, both hardware-free and independently
 *          unit-testable (no bus, no UART, no ring buffer needed to test
 *          either one — see tests/qemu_gps/main.c's "NMEA parser
 *          (unit-level, no bus)" section, which is exactly the
 *          "NMEA parser unit-tested against sample sentences" deliverable
 *          the task's suggested timeline calls out as doable in parallel
 *          with Members A/B, before any bus code exists):
 *
 *          1. **Framer** (`NMEA_Parser_t` + `NMEA_Feed()`): consumes one
 *             raw byte at a time (exactly what a polling UART RX loop
 *             produces), finds sentence boundaries ('$' ... checksum ...
 *             CR/LF), and validates the XOR checksum. Produces a
 *             null-terminated payload string with the '$', the '*hh'
 *             checksum, and the CR/LF all already stripped.
 *          2. **Field parser** (`NMEA_ParseSentence()` and the
 *             per-sentence-type functions it dispatches to): takes that
 *             payload string and decodes it into a `GPS_Data_t` (gps.h).
 */

#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "gps.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                          LAYER 1: SENTENCE FRAMER                          *
 *============================================================================*/

/** @brief NMEA0183 caps sentences at 82 characters including '$', the
 *         checksum, and CRLF. This buffer holds only the payload between
 *         '$' and '*' (see file header), so it never needs the full 82 —
 *         but sized with headroom rather than shaved to the theoretical
 *         minimum, since a garbled byte stream (the exact case this
 *         framer's checksum check exists to catch) could otherwise run
 *         past a razor-thin buffer before the framer notices anything is
 *         wrong. */
#define NMEA_MAX_PAYLOAD_LEN 96U

typedef enum {
    NMEA_FEED_PENDING = 0,     /**< Still accumulating; nothing to do yet. */
    NMEA_FEED_SENTENCE_READY,   /**< A checksum-valid sentence is ready in
                                     NMEA_Parser_t::payload. */
    NMEA_FEED_CHECKSUM_ERROR,   /**< Sentence terminated (or a checksum
                                     that couldn't possibly be valid was
                                     seen) but the XOR checksum did not
                                     match — discarded. */
    NMEA_FEED_OVERFLOW          /**< Sentence exceeded NMEA_MAX_PAYLOAD_LEN
                                     before terminating — discarded. Since
                                     this framer resyncs on the next '$'
                                     regardless (see NMEA_Feed()'s header
                                     comment), one oversized/corrupt
                                     sentence never wedges the parser. */
} NMEA_FeedResult_t;

/**
 * @brief Framer state. Handle-based like every other driver in this
 *        repo — no static/global mutable state (DRIVER_STANDARD.md
 *        Section 2), even though this isn't itself a
 *        DRIVER_STANDARD-shaped sensor driver.
 */
typedef struct NMEA_Parser {
    char    payload[NMEA_MAX_PAYLOAD_LEN + 1]; /**< Null-terminated once a
                                                     sentence is READY. */
    uint8_t payload_len;

    bool    in_sentence;    /**< Seen '$', haven't yet seen CR/LF. */
    bool    in_checksum;    /**< Seen '*', now consuming exactly 2 hex
                                  digits before CR/LF. */
    uint8_t checksum_digits_read;
    uint8_t running_checksum;   /**< XOR of every byte between '$' and
                                      '*' (NMEA0183's checksum
                                      definition) exclusive of both. */
    uint8_t received_checksum;  /**< Parsed from the 2 ASCII hex digits
                                      after '*'. */
    bool    overflowed;         /**< Set once payload_len would exceed
                                      NMEA_MAX_PAYLOAD_LEN; stays set
                                      (discarding further bytes) until the
                                      next '$' or terminator. */
} NMEA_Parser_t;

/** @brief Zeroes all framer state. Safe to call mid-sentence to abandon
 *         whatever was being accumulated (used by GPS_Reset()). */
void NMEA_Parser_Init(NMEA_Parser_t *parser);

/**
 * @brief Feeds one byte into the framer.
 * @details Seeing '$' always (re)starts a new sentence, even mid-frame —
 *          a real UART link can drop a byte, in which case the previous
 *          sentence was already unrecoverable; resyncing on the next '$'
 *          is the standard, simple recovery strategy and matches how
 *          real GPS-parsing firmware behaves.
 * @retval NMEA_FEED_PENDING on every byte except the one that completes
 *         or invalidates a sentence.
 * @retval NMEA_FEED_SENTENCE_READY — parser->payload is a valid,
 *         null-terminated sentence body ready for NMEA_ParseSentence().
 * @retval NMEA_FEED_CHECKSUM_ERROR / NMEA_FEED_OVERFLOW — sentence
 *         discarded; parser is ready for the next '$' with no further
 *         action needed from the caller.
 */
NMEA_FeedResult_t NMEA_Feed(NMEA_Parser_t *parser, uint8_t byte);

/*============================================================================*
 *                        LAYER 2: SENTENCE FIELD PARSER                      *
 *============================================================================*/

typedef enum {
    NMEA_SENTENCE_UNRECOGNIZED = 0, /**< Well-formed, checksum-valid, but
                                          not a type this driver decodes
                                          (GSA/GSV/VTG/GLL/ZDA/...). */
    NMEA_SENTENCE_GGA,
    NMEA_SENTENCE_RMC
} NMEA_SentenceType_t;

/**
 * @brief Identifies a payload's sentence type from its 5-character header
 *        field (2-char talker ID + 3-char sentence ID, e.g. "GPGGA",
 *        "GNRMC") without needing to know which specific talker ID the
 *        module uses — this driver treats GPGGA/GNGGA/GLGGA/GAGGA/BDGGA/
 *        ... all identically, matching every real multi-constellation
 *        receiver's actual output.
 */
NMEA_SentenceType_t NMEA_IdentifySentence(const char *payload);

/**
 * @brief Extracts the 2-character talker ID from a payload's header
 *        field (e.g. "GP" from "GPGGA,..."). Used by GPS_ReadDeviceID().
 * @retval true if payload is at least 5 characters (a valid header could
 *         be present); false otherwise. Does not itself validate that
 *         payload[2..4] is a recognized sentence type — call
 *         NMEA_IdentifySentence() for that.
 */
bool NMEA_ExtractTalkerId(const char *payload, char talker_id_out[2]);

/**
 * @brief Dispatches to NMEA_ParseGGA() / NMEA_ParseRMC() based on
 *        NMEA_IdentifySentence(), and updates fix->talker_id and the
 *        rolling sentences_parsed/unsupported_sentences counters
 *        regardless of type.
 * @param[in]     payload   A framer-validated payload (see NMEA_Feed()).
 * @param[in,out] fix       Updated in place — only the fields the
 *                          identified sentence type actually carries are
 *                          touched; see gps.h's GPS_Data_t comment for
 *                          why fields are merged rather than replaced
 *                          wholesale on every call.
 * @retval NMEA_SENTENCE_GGA / NMEA_SENTENCE_RMC if decoded and fix was
 *         updated.
 * @retval NMEA_SENTENCE_UNRECOGNIZED if the type isn't one this driver
 *         decodes (fix->talker_id and fix->unsupported_sentences are
 *         still updated; nothing else is touched).
 */
NMEA_SentenceType_t NMEA_ParseSentence(const char *payload, GPS_Data_t *fix);

/** @brief Decodes a $--GGA payload (talker+"GGA" header already
 *         consumed by the caller is NOT assumed — pass the full payload
 *         starting at the talker ID; this function skips the header
 *         field itself). See NMEA_ParseSentence() for the merge-not-
 *         replace field-update contract.
 * @retval true if the sentence had enough comma-delimited fields to be a
 *         plausible GGA (malformed/truncated sentences that still passed
 *         checksum are rejected rather than partially applied). */
bool NMEA_ParseGGA(const char *payload, GPS_Data_t *fix);

/** @brief Decodes a $--RMC payload. See NMEA_ParseGGA()'s comment. */
bool NMEA_ParseRMC(const char *payload, GPS_Data_t *fix);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NMEA_H */
