/**
 * @file    gps.h
 * @brief   Public API for a generic NMEA0183 GPS/GNSS module driver.
 * @details Handle-based, hardware-agnostic driver for the S09 ANSA HAL
 *          Emulator, following DRIVER_STANDARD.md. All bus traffic goes
 *          through the GPS_BusReadByte / GPS_BusWrite / GPS_DelayMs
 *          contract in gps_bus.h — see that file for why a byte-stream
 *          UART device gets a slightly different bus contract shape than
 *          the register-addressed I2C/SPI sensors (BMP388, ICM42688).
 *
 * @note    Part of the ANSA Hardware Abstraction Layer Driver Framework.
 *          Depends only on the standard library and this driver's own
 *          headers (gps_bus.h, gps_nmea.h, gps_ringbuffer.h).
 *
 *          Team context (Phase2_BareMetal_Task_Division.pdf): this is
 *          Member C's deliverable — UART peripherals & GPS — the third
 *          leg of the same S09 sensor-driver task that already produced
 *          Drivers/ICM42688 (Member B: SPI) and Drivers/HMC5883L /
 *          Drivers/BMP388 (Member A: I2C, plus BMP388 optionally SPI).
 *          See the repo root README.md's "Sensor drivers" table.
 */

#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              STATUS CODES                                  *
 *============================================================================*/

/**
 * @brief GPS driver status/return codes (DRIVER_STANDARD.md Section 5).
 */
typedef enum {
    GPS_OK = 0,                    /**< Operation succeeded. */
    GPS_ERROR_BUS,                  /**< UART transaction failed (hard bus
                                          fault — not the routine "no byte
                                          waiting yet" case, see GPS_Process()). */
    GPS_ERROR_INVALID_PARAM,        /**< Null pointer or bad handle field. */
    GPS_ERROR_NOT_READY,            /**< No valid position fix parsed yet
                                          (cold start, or no satellites in
                                          view) — this is normal, expected
                                          GPS behavior for up to tens of
                                          seconds after power-up, not a
                                          fault condition to alarm on. */
    GPS_ERROR_TIMEOUT,              /**< Reserved for future use by a
                                          blocking "wait for first fix"
                                          helper; not returned by any
                                          function in the current API
                                          surface, all of which are
                                          non-blocking. */
    GPS_ERROR_NOT_INITIALIZED,      /**< Called before GPS_Init(). */
    GPS_ERROR_CHECKSUM,              /**< A sentence arrived but its NMEA
                                          checksum did not match — the byte
                                          stream is corrupted or garbled,
                                          not that GPS itself has no fix.
                                          The sentence is discarded; the
                                          handle's last good fix (if any)
                                          is left untouched. */
    GPS_ERROR_UNSUPPORTED_SENTENCE  /**< The device sent a well-formed,
                                          checksum-valid sentence this
                                          driver doesn't decode (e.g. GSA,
                                          GSV, VTG) — informational, not a
                                          fault; GPS_ReadData() does not
                                          return this, only the lower-level
                                          NMEA_ParseSentence() does. */
} GPS_Status_t;

/*============================================================================*
 *                         DATA STRUCTURES & HANDLE                           *
 *============================================================================*/

/** @brief GGA fix-quality codes (NMEA0183 field 6 of $--GGA), reproduced
 *         verbatim rather than collapsed into a simplified "has fix"
 *         bool, since "DGPS fix" vs "GPS fix" vs "no fix" is exactly the
 *         kind of distinction a flight-control consumer of this data
 *         needs and a driver has no business hiding. */
typedef enum {
    GPS_FIX_QUALITY_INVALID       = 0,
    GPS_FIX_QUALITY_GPS           = 1,
    GPS_FIX_QUALITY_DGPS          = 2,
    GPS_FIX_QUALITY_PPS           = 3,
    GPS_FIX_QUALITY_RTK           = 4,
    GPS_FIX_QUALITY_FLOAT_RTK     = 5,
    GPS_FIX_QUALITY_ESTIMATED     = 6,
    GPS_FIX_QUALITY_MANUAL        = 7,
    GPS_FIX_QUALITY_SIMULATION    = 8
} GPS_FixQuality_t;

/**
 * @brief One consolidated GPS/GNSS fix, assembled from whichever of
 *        $--GGA (position/altitude/fix-quality/satellite count/HDOP) and
 *        $--RMC (speed/course/date, plus a second independent position +
 *        validity check) sentences have been seen so far. Fields not yet
 *        supplied by either sentence keep their previous value rather
 *        than resetting to zero — a real receiver interleaves GGA/RMC/
 *        GSA/GSV every fix cycle, and this struct is meant to always
 *        reflect "the best currently-known picture," not "only what the
 *        single most recent sentence happened to contain."
 */
typedef struct {
    double  latitude_deg;    /**< + = North, - = South. 0.0 before any fix. */
    double  longitude_deg;   /**< + = East, - = West. 0.0 before any fix. */
    double  altitude_m;      /**< Mean-sea-level altitude, from GGA field 9. */
    double  speed_knots;     /**< Ground speed, from RMC field 7. */
    double  course_deg;      /**< True ground track, from RMC field 8. */
    double  hdop;            /**< Horizontal dilution of precision, GGA field 8. */

    GPS_FixQuality_t fix_quality;   /**< GGA field 6. */
    uint8_t satellites_in_use;      /**< GGA field 7. */

    uint8_t  utc_hour;
    uint8_t  utc_minute;
    float    utc_second;     /**< Fractional seconds as reported (GGA/RMC
                                   both carry hhmmss.sss to the same
                                   precision). */
    uint8_t  utc_day;        /**< 1-31, from RMC field 9 (ddmmyy). 0 if
                                   no RMC sentence has been seen yet. */
    uint8_t  utc_month;      /**< 1-12. */
    uint16_t utc_year;       /**< Expanded to 20xx — see gps_nmea.c's
                                   NMEA_ParseRMC() header comment for why
                                   that Y2K-style assumption is acceptable
                                   here and how to revisit it. */

    bool     rmc_status_valid; /**< RMC field 2: 'A' = data valid, 'V' =
                                     receiver warning (no fix). Kept
                                     alongside fix_quality rather than
                                     collapsed into it, since GGA and RMC
                                     are independent sentences that can
                                     (rarely, momentarily) disagree. */

    char     talker_id[3];   /**< "GP"/"GN"/"GL"/"GA"/"BD"/"QZ"/... plus a
                                   null terminator — which constellation
                                   the most recently parsed sentence came
                                   from. See GPS_ReadDeviceID(). */

    uint32_t sentences_parsed;   /**< Rolling count of checksum-valid,
                                       recognized (GGA/RMC) sentences. */
    uint32_t checksum_errors;    /**< Rolling count — see GPS_ERROR_CHECKSUM. */
    uint32_t unsupported_sentences; /**< Rolling count of checksum-valid
                                          sentences of a type this driver
                                          doesn't decode (GSA/GSV/VTG/...). */
} GPS_Data_t;

/**
 * @brief Master handle for one GPS module instance. No static or global
 *        mutable state lives in gps.c (DRIVER_STANDARD.md Section 2) —
 *        the ring buffer, parser state, and latest fix all live here.
 */
typedef struct {
    void    *bus_context;   /**< Opaque platform bus context, passed to
                                  every GPS_Bus* call unchanged. */
    uint32_t timeout_ms;    /**< Reserved for a future blocking helper;
                                  not read by the current API (see
                                  GPS_ERROR_TIMEOUT's comment). */

    struct GPS_RingBuffer   *rx_ring;   /**< Opaque forward declaration —
                                              see gps_ringbuffer.h for the
                                              real definition. Kept
                                              opaque here so gps.h itself
                                              stays a minimal public
                                              surface; gps.c owns the
                                              concrete storage. */
    struct NMEA_Parser      *nmea;      /**< Opaque — see gps_nmea.h. */

    GPS_Data_t last_fix;     /**< Most recent consolidated fix. */

    bool     initialized;
    uint32_t fault_count;    /**< Rolling GPS_ERROR_BUS counter. */
    GPS_Status_t last_error;
} GPS_Handle_t;

/*============================================================================*
 *                          PUBLIC API — REQUIRED SET                         *
 *============================================================================*/

/**
 * @brief Brings up the handle: resets the ring buffer and NMEA parser
 *        state, zeroes last_fix. Deliberately does **not** block waiting
 *        for a first sentence or fix — real GPS modules can take 30+
 *        seconds to acquire a cold-start fix, and DRIVER_STANDARD.md's
 *        `<S>_Init()` contract ("verify identity") has no clean analogue
 *        for a device with no WHO_AM_I register; call GPS_ReadData()
 *        afterwards (repeatedly, e.g. once per main-loop tick) and expect
 *        GPS_ERROR_NOT_READY until the first valid sentence arrives. See
 *        `Drivers/GPS/README.md` "Deviations from DRIVER_STANDARD.md".
 * @param[in,out] handle  Device instance. bus_context/timeout_ms must
 *                        already be set by the caller; rx_ring/nmea must
 *                        point at caller-owned storage (see
 *                        GPS_Handle_t's field comments) — this driver
 *                        performs no dynamic allocation.
 * @retval GPS_OK on success.
 * @retval GPS_ERROR_INVALID_PARAM if handle, handle->rx_ring, or
 *         handle->nmea is null.
 */
GPS_Status_t GPS_Init(GPS_Handle_t *handle);

/**
 * @brief Marks the handle de-initialized. Does not touch the UART
 *        peripheral itself — see gps_bus.h's GPS_BusDeinit() note for why
 *        that's a backend concern, not this driver's.
 */
GPS_Status_t GPS_DeInit(GPS_Handle_t *handle);

/**
 * @brief The closest UART/NMEA analogue to a WHO_AM_I register: reports
 *        the 2-character NMEA talker ID (see GPS_Data_t::talker_id) seen
 *        on the most recently parsed sentence — "GP" for GPS-only,
 *        "GN" for a multi-constellation receiver, "GL"/"GA"/"BD"/"QZ" for
 *        GLONASS/Galileo/BeiDou/QZSS-only fixes on a module that reports
 *        those separately. This is a real, meaningful identity signal
 *        (it tells you what kind of receiver is actually on the wire),
 *        just not a fixed register value the way HMC5883L's "H43" or
 *        BMP388's 0x50 CHIP_ID are.
 * @param[in,out] handle    Device instance.
 * @param[out]    id_out    Destination for 2 ASCII characters (not null
 *                          terminated by this call — pass a 2-byte
 *                          buffer, or read GPS_Data_t::talker_id directly
 *                          via GPS_ReadData() if a null terminator is
 *                          wanted).
 * @retval GPS_OK on success.
 * @retval GPS_ERROR_NOT_READY if no sentence has been parsed yet (talker
 *         ID is unknown, not just "no fix yet").
 */
GPS_Status_t GPS_ReadDeviceID(GPS_Handle_t *handle, char id_out[2]);

/**
 * @brief Soft-resets driver state: clears the ring buffer and NMEA
 *        parser's in-progress sentence, and zeroes last_fix and its
 *        rolling counters. Does not touch UART hardware configuration —
 *        symmetric with GPS_Init() (see its comment) and with
 *        DRIVER_STANDARD.md Section 3's "Reset() ... device only"
 *        framing, adapted here to "reset driver-side state only," since
 *        there is no device-side register to reset over a plain NMEA
 *        UART link.
 */
GPS_Status_t GPS_Reset(GPS_Handle_t *handle);

/**
 * @brief Drains every byte currently waiting on the bus into the ring
 *        buffer, feeds the ring buffer through the NMEA parser, applies
 *        any complete valid sentences to handle->last_fix, and copies
 *        the resulting snapshot out.
 * @details This single call does the work GPS_Process() + "read the
 *          struct" would do separately — provided as one call because
 *          DRIVER_STANDARD.md Section 3 requires a `<S>_ReadData()` with
 *          exactly this shape. Call GPS_Process() directly instead if a
 *          caller wants to pump the parser from an RX interrupt context
 *          without also copying out a snapshot every time.
 * @param[in,out] handle  Initialized device instance.
 * @param[out]    data    Destination for the current best-known fix.
 * @retval GPS_OK on success (data populated; may still be a stale fix if
 *         nothing new validated this call — check
 *         data->sentences_parsed against a previous call's value if
 *         "did anything new arrive" matters to the caller).
 * @retval GPS_ERROR_NOT_INITIALIZED if GPS_Init() has not succeeded yet.
 * @retval GPS_ERROR_NOT_READY if no valid GGA/RMC sentence has ever been
 *         parsed (handle->last_fix is still all-zero/default).
 * @retval GPS_ERROR_BUS if the underlying GPS_BusReadByte() reports a
 *         hard fault (not the routine "no byte available" case).
 */
GPS_Status_t GPS_ReadData(GPS_Handle_t *handle, GPS_Data_t *data);

/*============================================================================*
 *                       PUBLIC API — SENSOR-SPECIFIC EXTRAS                  *
 *============================================================================*/

/**
 * @brief Pumps the pipeline (bus -> ring buffer -> NMEA parser ->
 *        last_fix) without copying anything out. Intended for a caller
 *        that wants to keep the parser warm on every main-loop tick (or
 *        from a UART RX-ready interrupt handler, since every step here is
 *        non-blocking and bounded — see gps_bus.h's GPS_BusReadByte()
 *        contract) and only calls GPS_ReadData() when it actually needs a
 *        snapshot.
 * @retval GPS_OK, or GPS_ERROR_BUS / GPS_ERROR_NOT_INITIALIZED as in
 *         GPS_ReadData().
 */
GPS_Status_t GPS_Process(GPS_Handle_t *handle);

/**
 * @brief Sends a raw byte string over the same UART link — the task
 *        brief's "support any OSD control via UART if needed (some OSDs
 *        accept serial commands)." This driver does not implement any
 *        specific OSD's command protocol (out of scope: the brief itself
 *        says "if needed", and no specific OSD chip/protocol was named
 *        for the UART path — MAX7456/AT7456E-family OSDs, the ones
 *        actually named in Phase2_BareMetal_Task_Division.pdf, are SPI
 *        parts owned by Member B's layer, not this one). What this
 *        function provides is the one genuinely reusable piece: a raw,
 *        already-tested byte-write path over the bare-metal UART bus, so
 *        whoever integrates a serial-controlled OSD later doesn't have
 *        to write their own UART transmit function from scratch.
 * @param[in,out] handle  Initialized device instance.
 * @param[in]     data    Raw bytes to send.
 * @param[in]     length  Number of bytes.
 * @retval GPS_OK on success; GPS_ERROR_BUS on a transmit failure.
 */
GPS_Status_t GPS_SendRawCommand(GPS_Handle_t *handle, const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
