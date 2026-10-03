/**
 ******************************************************************************
 * @file    bmp388.c
 * @brief   Implementation of the Bosch BMP388 barometric pressure and
 *          temperature driver declared in bmp388.h.
 * @details
 *   Pipeline: BMP388_GetData() -> BMP388_ReadRawData() (SPI burst) ->
 *             compensate (datasheet formulas) -> BMP388_Data.
 *
 *   All bus traffic goes through the shared bare-metal SPI1 driver in
 *   Protocols/SPI/ (spi_driver.h). The BMP388 shares SPI1 with the
 *   ICM20948, so every transaction passes this driver's chip-select pin
 *   (BMP388_CS_PIN, PA4) to SPI1_ReadRegisters() / SPI1_WriteRegister().
 *
 *   No static or global mutable state: every function is either pure or
 *   operates on caller-supplied structs.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

#include "bmp388.h"
#include "bmp388_private.h"
#include "bmp388_config.h"
#include "spi_driver.h"

/*============================================================================*
 *                          INTERNAL CONSTANTS                                *
 *============================================================================*/

/**
 * @brief Largest single burst this driver ever reads.
 *
 * @details
 *   The two bursts in this driver are 6 bytes (sensor data) and 21 bytes
 *   (calibration). The +1 slack is for the dummy byte the BMP388 in SPI
 *   mode requires between the address byte and the data — see
 *   BMP388_ReadRegs()'s comment. 32 is comfortably above both.
 */
#define BMP388_MAX_READ_LEN   32U

/*============================================================================*
 *                          INTERNAL BUS WRAPPERS                             *
 *============================================================================*/

/**
 * @brief BMP388-specific register read: BMP388 SPI needs one dummy byte.
 *
 * @details
 *   The protocol-layer SPI1_ReadRegisters() sends the address byte and
 *   then clocks out `length` bytes immediately. The BMP388 does not
 *   accept that — its SPI state machine inserts one dummy byte between
 *   the address and the first data byte, so the naive call returns the
 *   data shifted one position to the left (and one byte short at the
 *   end).
 *
 *   The fix is to read one byte more than we actually want and discard
 *   the first one. That's why the caller never sees this function —
 *   the rest of the driver just calls BMP388_ReadRegs() and gets the
 *   correct data.
 *
 * @param[in]  reg_addr  Register address byte (already ORed with 0x80
 *                       if this is a read — see the callers below).
 * @param[out] buffer    Destination for the real data bytes.
 * @param[in]  length    Number of real data bytes to return.
 */
static void BMP388_ReadRegs(uint8_t reg_addr, uint8_t *buffer, uint16_t length)
{
    uint8_t temp[BMP388_MAX_READ_LEN + 1U];

    /* Request length + 1 bytes so the dummy byte lands in temp[0]. */
    SPI1_ReadRegisters(BMP388_CS_PIN, reg_addr | 0x80U, temp, (uint16_t)(length + 1U));

    /* Shift out the dummy byte. */
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = temp[i + 1U];
    }
}

/**
 * @brief BMP388-specific register write.
 *
 * @details
 *   No dummy byte on the write path — only reads have the extra
 *   turnaround phase. Bit 7 is cleared because the BMP388 uses it as
 *   the read/write selector (1 = read, 0 = write), same as the
 *   ICM20948.
 *
 * @param[in] reg_addr  Register address (bit 7 is forced to 0).
 * @param[in] value     Byte to write.
 */
static void BMP388_WriteReg(uint8_t reg_addr, uint8_t value)
{
    SPI1_WriteRegister(BMP388_CS_PIN, (uint8_t)(reg_addr & 0x7FU), value);
}

/*============================================================================*
 *                          INTERNAL DELAY                                    *
 *============================================================================*/

/**
 * @brief Short busy-wait used during reset.
 *
 * @details
 *   The BMP388 needs a few milliseconds after a soft reset before it
 *   answers on the bus again. This project has a proper timer-based
 *   delay coming (TIM_interface.h / TIM_program.c, TIM6), and once it
 *   exists this loop should be replaced by a call to it. Until then,
 *   an uncalibrated spin is what's available — the driver's README
 *   documents this as a known simplification.
 *
 *   Over-waiting is harmless here (the part is ready well before this
 *   returns), under-waiting would produce a WHO_AM_I failure on some
 *   silicon corners, so the loop is deliberately generous.
 *
 * @param[in] iters  Loop iteration count — not calibrated to real time.
 */
static void BMP388_DelayApprox(volatile uint32_t iters)
{
    while (iters--) {
        __asm__("nop");
    }
}

/*============================================================================*
 *                          PUBLIC API IMPLEMENTATION                         *
 *============================================================================*/

uint8_t BMP388_ReadChipID(void)
{
    uint8_t id = 0U;
    BMP388_ReadRegs(BMP388_REG_CHIP_ID, &id, 1U);
    return id;
}

void BMP388_SoftReset(void)
{
    BMP388_WriteReg(BMP388_REG_CMD, BMP388_SOFT_RESET_CMD);
}

void BMP388_SetPowerCtrl(uint8_t value)
{
    BMP388_WriteReg(BMP388_REG_PWR_CTRL, value);
}

void BMP388_SetOSR(uint8_t osr_p, uint8_t osr_t)
{
    BMP388_WriteReg(BMP388_REG_OSR, BMP388_OSR_BYTE(osr_p, osr_t));
}

void BMP388_SetODR(uint8_t odr)
{
    BMP388_WriteReg(BMP388_REG_ODR, (uint8_t)(odr & 0x1FU));
}

void BMP388_ReadCalibData(BMP388_CalibData *calib)
{
    if (calib == 0) {
        return;
    }

    uint8_t raw[21];
    BMP388_ReadRegs(BMP388_REG_CALIB_DATA, raw, 21U);

    /* Calibration words are little-endian on the BMP388 (unlike the
     * ICM20948, which is big-endian). Field layout is:
     *
     *   [0..1]   par_t1   (unsigned 16)
     *   [2..3]   par_t2   (signed   16)
     *   [4]      par_t3   (signed    8)
     *   [5..6]   par_p1   (signed   16)
     *   [7..8]   par_p2   (signed   16)
     *   [9]      par_p3   (signed    8)
     *   [10]     par_p4   (signed    8)
     *   [11..12] par_p5   (unsigned 16)
     *   [13..14] par_p6   (unsigned 16)
     *   [15]     par_p7   (signed    8)
     *   [16]     par_p8   (signed    8)
     *   [17..18] par_p9   (signed   16)
     *   [19]     par_p10  (signed    8)
     *   [20]     par_p11  (signed    8)
     */
    uint16_t par_t1_raw  = (uint16_t)((uint16_t)raw[1]  << 8 | (uint16_t)raw[0]);
    int16_t  par_t2_raw  = (int16_t )((uint16_t)raw[3]  << 8 | (uint16_t)raw[2]);
    int8_t   par_t3_raw  = (int8_t)raw[4];

    int16_t  par_p1_raw  = (int16_t )((uint16_t)raw[6]  << 8 | (uint16_t)raw[5]);
    int16_t  par_p2_raw  = (int16_t )((uint16_t)raw[8]  << 8 | (uint16_t)raw[7]);
    int8_t   par_p3_raw  = (int8_t)raw[9];
    int8_t   par_p4_raw  = (int8_t)raw[10];
    uint16_t par_p5_raw  = (uint16_t)((uint16_t)raw[12] << 8 | (uint16_t)raw[11]);
    uint16_t par_p6_raw  = (uint16_t)((uint16_t)raw[14] << 8 | (uint16_t)raw[13]);
    int8_t   par_p7_raw  = (int8_t)raw[15];
    int8_t   par_p8_raw  = (int8_t)raw[16];
    int16_t  par_p9_raw  = (int16_t )((uint16_t)raw[18] << 8 | (uint16_t)raw[17]);
    int8_t   par_p10_raw = (int8_t)raw[19];
    int8_t   par_p11_raw = (int8_t)raw[20];

    /* Scale each raw coefficient by its datasheet power-of-two divisor,
     * and — for p1 and p2 — remove the +16384 bias first. The named
     * constants all live in bmp388_private.h so the math below reads
     * like the datasheet instead of a forest of magic numbers. */
    calib->par_t1  = (float)par_t1_raw / BMP388_CALIB_T1_DIV;
    calib->par_t2  = (float)par_t2_raw / BMP388_CALIB_T2_DIV;
    calib->par_t3  = (float)par_t3_raw / BMP388_CALIB_T3_DIV;

    calib->par_p1  = ((float)par_p1_raw - BMP388_CALIB_P1_BIAS) / BMP388_CALIB_P1_DIV;
    calib->par_p2  = ((float)par_p2_raw - BMP388_CALIB_P2_BIAS) / BMP388_CALIB_P2_DIV;
    calib->par_p3  = (float)par_p3_raw  / BMP388_CALIB_P3_DIV;
    calib->par_p4  = (float)par_p4_raw  / BMP388_CALIB_P4_DIV;
    calib->par_p5  = (float)par_p5_raw  / BMP388_CALIB_P5_DIV;
    calib->par_p6  = (float)par_p6_raw  / BMP388_CALIB_P6_DIV;
    calib->par_p7  = (float)par_p7_raw  / BMP388_CALIB_P7_DIV;
    calib->par_p8  = (float)par_p8_raw  / BMP388_CALIB_P8_DIV;
    calib->par_p9  = (float)par_p9_raw  / BMP388_CALIB_P9_DIV;
    calib->par_p10 = (float)par_p10_raw / BMP388_CALIB_P10_DIV;
    calib->par_p11 = (float)par_p11_raw / BMP388_CALIB_P11_DIV;

    calib->t_lin   = 0.0f;
}

void BMP388_ReadRawData(uint32_t *raw_press, uint32_t *raw_temp)
{
    if (raw_press == 0 || raw_temp == 0) {
        return;
    }

    uint8_t raw[6];
    BMP388_ReadRegs(BMP388_REG_DATA_0, raw, 6U);

    /* Sensor data is little-endian: pressure is bytes [2:1:0], then
     * temperature is bytes [5:4:3]. Both are 24-bit unsigned. */
    *raw_press = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | (uint32_t)raw[0];
    *raw_temp  = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | (uint32_t)raw[3];
}

/*============================================================================*
 *                          COMPENSATION MATH                                 *
 *============================================================================*/

/**
 * @brief Temperature compensation (datasheet §9.3.1, "temperature").
 *
 * @details
 *   Writes the intermediate linearized temperature back into
 *   calib->t_lin so the pressure compensation can use it. That's the
 *   one non-obvious thing about this function — it is not pure, because
 *   Bosch's formulas are naturally a two-stage pipeline and threading
 *   the intermediate through a return value would obscure the
 *   correspondence with the datasheet.
 *
 * @param[in]  uncomp_temp  Raw 24-bit temperature count.
 * @param[in,out] calib     Calibration struct; t_lin is written.
 *
 * @return Linearized temperature [°C].
 */
static float BMP388_CompensateTemperature(uint32_t uncomp_temp, BMP388_CalibData *calib)
{
    float partial1 = (float)uncomp_temp - calib->par_t1;
    float partial2 = partial1 * calib->par_t2;

    calib->t_lin = partial2 + (partial1 * partial1) * calib->par_t3;

    return calib->t_lin;
}

/**
 * @brief Pressure compensation (datasheet §9.3.1, "pressure").
 *
 * @details
 *   Requires calib->t_lin to already hold the result of the temperature
 *   stage — that's why BMP388_GetData() calls the temperature function
 *   first even when the caller only wants pressure.
 *
 * @param[in] uncomp_press  Raw 24-bit pressure count.
 * @param[in] calib         Calibration struct; t_lin must be current.
 *
 * @return Compensated pressure [Pa].
 */
static float BMP388_CompensatePressure(uint32_t uncomp_press, const BMP388_CalibData *calib)
{
    float t_lin = calib->t_lin;

    float partial1 = calib->par_p6 * t_lin;
    float partial2 = calib->par_p7 * (t_lin * t_lin);
    float partial3 = calib->par_p8 * (t_lin * t_lin * t_lin);
    float partial_out1 = calib->par_p5 + partial1 + partial2 + partial3;

    partial1 = calib->par_p2 * t_lin;
    partial2 = calib->par_p3 * (t_lin * t_lin);
    partial3 = calib->par_p4 * (t_lin * t_lin * t_lin);
    float partial_out2 = (float)uncomp_press * (calib->par_p1 + partial1 + partial2 + partial3);

    float press2 = (float)uncomp_press * (float)uncomp_press;
    float partial_p9_p10 = calib->par_p9 + calib->par_p10 * t_lin;
    float partial4 = press2 * partial_p9_p10;
    partial4 += ((float)uncomp_press * press2) * calib->par_p11;

    return partial_out1 + partial_out2 + partial4;
}

/*============================================================================*
 *                          TOP-LEVEL PUBLIC API                              *
 *============================================================================*/

void BMP388_GetData(BMP388_CalibData *calib, BMP388_Data *out)
{
    if (calib == 0 || out == 0) {
        return;
    }

    uint32_t raw_press = 0U;
    uint32_t raw_temp  = 0U;

    BMP388_ReadRawData(&raw_press, &raw_temp);

    /* Temperature first: the pressure formula reads calib->t_lin. */
    out->temperature = BMP388_CompensateTemperature(raw_temp, calib);
    out->pressure    = BMP388_CompensatePressure(raw_press, calib);
}

uint8_t BMP388_Init(BMP388_CalibData *calib)
{
    if (calib == 0) {
        return 0U;
    }

    /* 1. Identity check — the single most useful bring-up failure mode. */
    if (BMP388_ReadChipID() != BMP388_CHIP_ID_VALUE) {
        return 0U;
    }

    /* 2. Soft reset, then wait for the part to come back. */
    BMP388_SoftReset();
    BMP388_DelayApprox(400000U);   /* ~2 ms at 100 MHz — see DelayApprox() */

    /* 3. Load factory calibration. Everything below this needs it. */
    BMP388_ReadCalibData(calib);

    return 1U;
}
