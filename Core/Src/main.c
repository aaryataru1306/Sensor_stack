/**
 ******************************************************************************
 * @file    main.c
 * @brief   Top-level application: brings up the STM32F411CEU6 Black Pill
 *          bare-metal peripheral tree, brings up each sensor driver, and
 *          runs the sensor-fusion loop.
 * @details
 *   Boot order (deliberate — see comments at each step):
 *
 *     1. SystemClock_Config()      — HSE 25 MHz -> PLL -> 100 MHz SYSCLK
 *     2. SPI1_Init()               — shared bus, both CS lines idle high
 *     3. ICM20948_Init()           — primary IMU, SPI1 CS = PA1
 *     4. BMP388_Init()             — barometer, SPI1 CS = PA4
 *     5. MPU6050_enumInit()        — secondary IMU, I2C1 (PB6/PB7)
 *     6. gps_bus_stm32_setup()     — USART2 (PA2/PA3) RCC + GPIO + UART
 *     7. GPS_Init()                — reset driver state, do not wait
 *     8. SF_InitFromAccel()        — seed the EKF attitude from gravity
 *
 *   The main loop:
 *     - Reads both IMUs every tick.
 *     - Feeds the ICM's gyro + accel into the attitude EKF.
 *     - Reads the BMP388 and feeds pressure into the altitude filter.
 *     - Polls the GPS, and — when a valid 3D fix is present — feeds
 *       its altitude into the altitude filter as a slow reference.
 *     - Leaves the fused results in file-static structs for the
 *       debugger to inspect.
 *
 *   Pacing is a busy-wait today. A real 100 Hz timer tick is a
 *   follow-up (TIM6 exists in the protocol layer for that purpose);
 *   the delay loop below is set so the effective rate is roughly
 *   100 Hz at 100 MHz SYSCLK, but it is not exact and should not be
 *   relied on for anything but bring-up.
 *
 * @note  Part of the ANSA Hardware Abstraction Layer Driver Framework,
 *        target STM32F411CEU6 (Black Pill).
 ******************************************************************************
 */

/*============================================================================*
 *                                  INCLUDES                                  *
 *============================================================================*/

#include "main.h"

/* Protocol layer */
#include "spi_driver.h"
#include "i2c_interface.h"
#include "uart_bare.h"

/* Sensor drivers */
#include "icm42688.h"
#include "icm20948.h"
#include "bmp388.h"
#include "MPU6050_interface.h"

/* GPS driver */
#include "gps.h"
#include "gps_bus.h"
#include "gps_nmea.h"
#include "gps_ringbuffer.h"

/* Sensor fusion */
#include "sensor_fusion.h"

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*
 *                          SYSTEM CLOCK CONFIGURATION                        *
 *============================================================================*
 *
 * RM0383 §6 ("Reset and clock control") for every register touched
 * below. Bare-metal — no HAL_RCC_OscConfig, no HAL_RCC_ClockConfig.
 *
 * Target: HSE 25 MHz -> PLL (M=25, N=200, P=2) -> SYSCLK 100 MHz
 *         AHB  /1  -> HCLK  100 MHz
 *         APB1 /2  -> PCLK1  50 MHz
 *         APB2 /1  -> PCLK2 100 MHz
 *
 * If the Black Pill's HSE crystal isn't 25 MHz on your board (some
 * clones ship 8 MHz), change PLLM below and update the HSE_VALUE
 * define in the project preprocessor. The PLLN/PLLP that produce a
 * 200 MHz VCO and a 100 MHz SYSCLK stay the same.
 */

#define RCC_BASE             0x40023800UL
#define RCC_CR               (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_PLLCFGR          (*(volatile uint32_t *)(RCC_BASE + 0x04UL))
#define RCC_CFGR             (*(volatile uint32_t *)(RCC_BASE + 0x08UL))

#define FLASH_BASE           0x40023C00UL
#define FLASH_ACR            (*(volatile uint32_t *)(FLASH_BASE + 0x00UL))

#define PWR_BASE             0x40007000UL
#define PWR_CR               (*(volatile uint32_t *)(PWR_BASE + 0x00UL))

#define RCC_APB1ENR          (*(volatile uint32_t *)(RCC_BASE + 0x40UL))
#define RCC_APB1ENR_PWREN    (1U << 28)

#define RCC_CR_HSEON         (1U << 16)
#define RCC_CR_HSERDY        (1U << 17)
#define RCC_CR_PLLON         (1U << 24)
#define RCC_CR_PLLRDY        (1U << 25)

#define RCC_CFGR_SW_PLL      0x02U
#define RCC_CFGR_SWS_PLL     0x08U
#define RCC_CFGR_HPRE_DIV1   (0x0U << 4)
#define RCC_CFGR_PPRE1_DIV2  (0x4U << 10)
#define RCC_CFGR_PPRE2_DIV1  (0x0U << 13)

#define RCC_PLLCFGR_PLLSRC_HSE  (1U << 22)

/* 3 wait states + prefetch + I-cache + D-cache. Required for 100 MHz
 * at scale 1 (RM0383 §3.5.1). */
#define FLASH_ACR_100MHZ      0x00000705U

#define PWR_CR_VOS_SCALE1     (3U << 14)

void SystemClock_Config(void)
{
    /* 1. Enable PWR clock, then select regulator scale 1. */
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;
    PWR_CR |= PWR_CR_VOS_SCALE1;

    /* 2. Flash latency. Must be set before raising the clock. */
    FLASH_ACR = FLASH_ACR_100MHZ;

    /* 3. HSE on, wait for ready. */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY)) {
        /* spin */
    }

    /* 4. PLL: HSE source, M=25, N=200, P=2 (SYSCLK = 100 MHz). */
    RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
                | (25U  <<  0)    /* PLLM */
                | (200U <<  6)    /* PLLN */
                | (0U   << 16)    /* PLLP: 0 => /2 */
                | (4U   << 24);   /* PLLQ */

    /* 5. PLL on, wait for ready. */
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY)) {
        /* spin */
    }

    /* 6. Prescalers: AHB /1, APB1 /2, APB2 /1. */
    RCC_CFGR &= ~((0xFU << 4) | (0x7U << 10) | (0x7U << 13));
    RCC_CFGR |= RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_PPRE2_DIV1;

    /* 7. Switch SYSCLK to PLL, wait for the switch to take effect. */
    RCC_CFGR &= ~0x3U;
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC_CFGR & 0xCU) != RCC_CFGR_SWS_PLL) {
        /* spin */
    }
}

/*============================================================================*
 *                          STATIC INSTANCES                                  *
 *============================================================================*
 *
 * All of these live for the lifetime of the program. They are declared
 * file-static (not local to main) so a debugger can inspect them live,
 * and so main's own stack frame stays small. The EKF's internal state
 * (the 7×7 covariance matrix) lives inside sensor_fusion.c, not here —
 * the fusion layer is a single-instance module for now.
 */

/* --- Raw sensor readings --- */
static ICM42688_Data_t  g_icm42688_data;
static bool             g_has_icm42688 = false;
static ICM20948_Data_t   g_icm_data;
static BMP388_CalibData  g_bmp_calib;
static BMP388_Data       g_bmp_data;
static MPU6050_Data_t    g_mpu_data;

/* --- GPS driver storage (caller-owned; the driver does no allocation) --- */
static GPS_RingBuffer_t  g_gps_ring;
static NMEA_Parser_t     g_gps_parser;
static GPS_Handle_t      g_gps;
static GPS_Data_t        g_gps_fix;

/* --- Fused outputs --- */
static SF_Attitude_t     g_attitude;
static SF_Altitude_t     g_altitude;

/*============================================================================*
 *                          CONVENIENCE MACROS                                *
 *============================================================================*/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG_TO_RAD  (M_PI / 180.0f)

/** @brief Nominal loop period [s] used as dt for the EKF. Replace with a
 *         real timer measurement once TIM6 is driving the loop. */
#define LOOP_DT_S      0.01f

/** @brief GPS fix quality threshold — we only feed altitude when the
 *         receiver reports at least a plain GPS fix (not "invalid") and
 *         has enough satellites for a 3D solution. */
#define GPS_MIN_SATS_FOR_ALT   4U

/*============================================================================*
 *                                MAIN                                        *
 *============================================================================*/

int main(void)
{
    /* ---- 1. Clock. ---- */
    SystemClock_Config();

    /* ---- 2. SPI1 (shared by ICM IMU and BMP388). ---- */
    SPI1_Init();

    /* ---- 3. Primary IMU on SPI1 (ICM-42688-P preferred, fallback to ICM-20948). ---- */
    if (ICM42688_InitDefault() == ICM42688_OK) {
        g_has_icm42688 = true;
    } else if (ICM20948_Init(ICM20948_ACCEL_FS_2G,
                             ICM20948_GYRO_FS_250DPS) != ICM20948_OK) {
        Error_Handler();
    }

    /* ---- 4. BMP388 on SPI1. ---- */
    if (BMP388_Init(&g_bmp_calib) == 0U) {
        Error_Handler();
    }

    /* ---- 5. MPU6050 on I2C1 (PB6/PB7 by default). ---- */
    if (MPU6050_enumInit() != OK) {
        Error_Handler();
    }

    /* ---- 6. GPS on USART2 (PA2 = TX, PA3 = RX). ----
     *
     * Bare-metal USART2 peripheral and GPIO setup at 9600 baud (NEO-M8N default).
     * IMU CS is on PA1, freeing PA3 for USART2_RX. */
    gps_bus_stm32_setup(PCLK1_HZ, 9600U);

    g_gps.bus_context = 0;
    g_gps.timeout_ms  = 100U;
    g_gps.rx_ring     = (struct GPS_RingBuffer *)&g_gps_ring;
    g_gps.nmea        = (struct NMEA_Parser    *)&g_gps_parser;

    if (GPS_Init(&g_gps) != GPS_OK) {
        Error_Handler();
    }

    /* ---- 7. Prime the fusion EKF ----
     *
     * Take one IMU accel reading, so the initial attitude is correct
     * regardless of how the board is mounted. If this fails (board
     * vibrating at power-on, or the reading out of range), fall back
     * to identity — the filter will still converge over the first
     * few seconds as accel updates come in. */
    if (g_has_icm42688) {
        ICM42688_ReadAll(&g_icm42688_data);
        if (SF_InitFromAccel(g_icm42688_data.accel_x,
                             g_icm42688_data.accel_y,
                             g_icm42688_data.accel_z) != SF_OK) {
            (void)SF_Reset(0);
        }
    } else {
        ICM20948_ReadAll(&g_icm_data);
        if (SF_InitFromAccel(g_icm_data.accel_x,
                             g_icm_data.accel_y,
                             g_icm_data.accel_z) != SF_OK) {
            (void)SF_Reset(0);
        }
    }

    /* ---- 8. Main loop. ---- */
    for (;;) {
        /* === A. Read every sensor === */
        float gx_rad, gy_rad, gz_rad;
        float ax, ay, az;

        if (g_has_icm42688) {
            ICM42688_ReadAll(&g_icm42688_data);
            gx_rad = g_icm42688_data.gyro_rad_x;
            gy_rad = g_icm42688_data.gyro_rad_y;
            gz_rad = g_icm42688_data.gyro_rad_z;
            ax     = g_icm42688_data.accel_x;
            ay     = g_icm42688_data.accel_y;
            az     = g_icm42688_data.accel_z;
        } else {
            ICM20948_ReadAll(&g_icm_data);
            gx_rad = g_icm_data.gyro_x * DEG_TO_RAD;
            gy_rad = g_icm_data.gyro_y * DEG_TO_RAD;
            gz_rad = g_icm_data.gyro_z * DEG_TO_RAD;
            ax     = g_icm_data.accel_x;
            ay     = g_icm_data.accel_y;
            az     = g_icm_data.accel_z;
        }

        (void)MPU6050_enumReadData(&g_mpu_data);
        BMP388_GetData(&g_bmp_calib, &g_bmp_data);
        (void)GPS_ReadData(&g_gps, &g_gps_fix);

        /* === B. Attitude EKF ===
         *
         * Gyro in rad/s, accel in g. */
        (void)SF_Update(0,
                        gx_rad, gy_rad, gz_rad,
                        ax, ay, az,
                        LOOP_DT_S);

        (void)SF_GetAttitude(0, &g_attitude);

        /* === C. Altitude filter ===
         *
         * BMP388 pressure at the loop rate; GPS altitude as a slow
         * absolute reference, only when the fix is trustworthy. */
        (void)SF_AltitudeFeedBaro(0, g_bmp_data.pressure, LOOP_DT_S);

        if (g_gps_fix.fix_quality >= GPS_FIX_QUALITY_GPS
         && g_gps_fix.satellites_in_use >= GPS_MIN_SATS_FOR_ALT
         && g_gps_fix.rmc_status_valid) {
            (void)SF_AltitudeFeedGPS(0,
                                     (float)g_gps_fix.altitude_m,
                                     true);
        }

        (void)SF_GetAltitude(0, &g_altitude);

        /* === D. Loop pacing ===
         *
         * Rough ~100 Hz at 100 MHz SYSCLK. Replace with a real TIM6
         * interrupt or SysTick-driven wait once the protocol layer
         * grows a proper timer. The busy-wait is fine for bring-up;
         * it is *not* fine for anything that needs deterministic
         * timing (e.g. tight gyro integration over long runs). */
        for (volatile uint32_t i = 0U; i < 500000U; i++) {
            __asm__("nop");
        }
    }
}

/*============================================================================*
 *                          ERROR HANDLER                                     *
 *============================================================================*/

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
        /* spin — attach a debugger and inspect the stack to find which
         * init failed. The file-static g_* structs and the EKF's state
         * are all visible in the debugger's watch panel. */
    }
}
