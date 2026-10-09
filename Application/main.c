#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "uart.h"
#include "i2c.h"
#include "qmc5883p.h"

/* ============================================================
 * STM32F411CEU6 - QMC5883P MAGNETOMETER TEST (standalone)
 *
 * UART : USART1, PA9 TX, PA10 RX
 * I2C1 : PB6 SCL, PB7 SDA (AF4)
 * ============================================================ */

#define PCLK1_HZ            16000000U
#define PCLK2_HZ            16000000U
#define UART_BAUD           115200U
#define UART_TIMEOUT        200000U
#define LINE_BUF_SIZE       160U

/* Set to 1 to run a ~10 s min/max calibration at startup (rotate the board) */
#define DO_CALIBRATION      1
#define CALIB_SAMPLES       500U    /* 500 samples @ 50 Hz = 10 s */

/* Magnetic declination for your location in degrees (+ east). Pune is ~ +0.5 */
#define DECLINATION_DEG     0.5f

static UART_Handle_t console;
static i2c_t mag_i2c;
static qmc5883p_t mag;

/* Required by existing UART/SPI drivers for debugging */
UART_Handle_t g_debug_uart;

/* ============================================================
 * UART OUTPUT HELPERS
 * ============================================================ */

static void console_write(const char *buf, uint16_t len)
{
    (void)UART_Transmit(&console, (const uint8_t *)buf, len);
}

static void console_print(const char *s)
{
    uint16_t n = 0U;
    while (s[n] != '\0') {
        n++;
    }
    console_write(s, n);
}

static char *put_str(char *p, const char *s)
{
    while (*s != '\0') {
        *p++ = *s++;
    }
    return p;
}

static char *put_u32(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0U;

    do {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);

    while (n != 0U) {
        *p++ = tmp[--n];
    }
    return p;
}

static char *put_i32(char *p, int32_t v)
{
    if (v < 0) {
        *p++ = '-';
        return put_u32(p, (uint32_t)(-v));
    }
    return put_u32(p, (uint32_t)v);
}

static char *put_hex8(char *p, uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    *p++ = hex[(value >> 4) & 0x0FU];
    *p++ = hex[value & 0x0FU];
    return p;
}

static char *put_fixed3(char *p, float v)
{
    int32_t m = (int32_t)(v * 1000.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
    uint32_t a;

    if (m < 0) {
        *p++ = '-';
        a = (uint32_t)(-m);
    } else {
        *p++ = '+';
        a = (uint32_t)m;
    }

    p = put_u32(p, a / 1000U);
    *p++ = '.';
    *p++ = (char)('0' + ((a % 1000U) / 100U));
    *p++ = (char)('0' + ((a % 100U) / 10U));
    *p++ = (char)('0' + (a % 10U));
    return p;
}

static void print_code(const char *msg, int code)
{
    char line[LINE_BUF_SIZE];
    char *p = line;

    p = put_str(p, msg);
    p = put_i32(p, code);
    p = put_str(p, "\r\n");
    console_write(line, (uint16_t)(p - line));
}

/* ============================================================
 * UART PIN INITIALIZATION
 * ============================================================ */

static void uart_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    /* PA9 = USART1_TX, PA10 = USART1_RX, AF7. */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 18) | (3U << 20))) | (2U << 18) | (2U << 20);
    GPIOA->OTYPER &= ~((1U << 9) | (1U << 10));
    GPIOA->OSPEEDR |= (3U << 18) | (3U << 20);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U << 18) | (3U << 20))) | (1U << 20);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFU << 4) | (0xFU << 8))) | (7U << 4) | (7U << 8);
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0U) {
        for (volatile uint32_t i = 0U; i < 4000U; i++) {
            __NOP();
        }
    }
}

/* ============================================================
 * DIAGNOSTICS
 * ============================================================ */

static void scan_i2c_bus(void)
{
    uint8_t found[16];
    uint8_t count;

    console_print("\r\n--- I2C BUS SCAN ---\r\n");
    count = i2c_scan(&mag_i2c, found, 16U);

    if (count == 0U) {
        console_print("No I2C devices found!\r\n");
        return;
    }
    for (uint8_t i = 0U; i < count; i++) {
        char line[LINE_BUF_SIZE];
        char *p = line;
        p = put_str(p, "Found device at 0x");
        p = put_hex8(p, found[i]);
        p = put_str(p, (found[i] == QMC5883P_I2C_ADDR) ? "  <-- QMC5883P\r\n" : "\r\n");
        console_write(line, (uint16_t)(p - line));
    }
}

static void print_reg(const char *name, uint8_t reg)
{
    uint8_t v = 0U;
    char line[LINE_BUF_SIZE];
    char *p = line;

    if (i2c_read_reg(&mag_i2c, QMC5883P_I2C_ADDR, reg, &v) != I2C_OK) {
        console_print("  reg read failed\r\n");
        return;
    }
    p = put_str(p, name);
    p = put_str(p, " = 0x");
    p = put_hex8(p, v);
    p = put_str(p, "\r\n");
    console_write(line, (uint16_t)(p - line));
}

static void dump_regs(void)
{
    print_reg("CHIP_ID", QMC5883P_REG_CHIP_ID);
    print_reg("STATUS ", QMC5883P_REG_STATUS);
    print_reg("CTRL1  ", QMC5883P_REG_CTRL1);
    print_reg("CTRL2  ", QMC5883P_REG_CTRL2);
}

/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    UART_Config_t ucfg;
    i2c_config_t icfg;
    qmc5883p_config_t mcfg;
    qmc5883p_data_t data;
    qmc5883p_status_t st;

    /* Step 1: UART */
    uart_pins_init();

    ucfg.peripheral_clock_hz = PCLK2_HZ;
    ucfg.baud_rate = UART_BAUD;
    ucfg.timeout_iters = UART_TIMEOUT;

    if (UART_Init(&console, (volatile uint32_t *)USART1_BASE, &ucfg) != UART_OK) {
        for (;;) { __NOP(); }
    }
    g_debug_uart = console;

    console_print(
        "\r\n========================================\r\n"
        "       QMC5883P MAGNETOMETER TEST       \r\n"
        "========================================\r\n"
        "MCU       : STM32F411CEU6\r\n"
        "UART      : USART1, 115200 baud\r\n"
        "I2C       : I2C1, PB6 (SCL), PB7 (SDA)\r\n"
        "Speed     : 400 kHz Fast Mode\r\n"
        "========================================\r\n"
    );

    /* Step 2: I2C1 */
    console_print("\r\n[1] Initializing I2C1...\r\n");

    icfg.instance = I2C1;
    icfg.scl_port = GPIOB;
    icfg.scl_pin = 6;
    icfg.sda_port = GPIOB;
    icfg.sda_pin = 7;
    icfg.af = 4;
    icfg.internal_pullup = true;
    icfg.pclk1_hz = PCLK1_HZ;
    icfg.speed_hz = 400000;
    icfg.timeout = 10000;

    if (i2c_init(&mag_i2c, &icfg) != I2C_OK) {
        console_print("I2C initialization failed!\r\n");
        for (;;) { delay_ms(1000U); }
    }

    /* Step 3: Scan */
    scan_i2c_bus();

    if (i2c_probe(&mag_i2c, QMC5883P_I2C_ADDR) != I2C_OK) {
        console_print("QMC5883P not detected at 0x2C! Check wiring/power.\r\n");
        for (;;) { delay_ms(1000U); }
    }

    /* Step 4: Driver init */
    console_print("\r\n[2] Initializing QMC5883P driver...\r\n");

    qmc5883p_get_default_config(&mcfg);
    mcfg.range = QMC5883P_RANGE_8G;
    mcfg.odr   = QMC5883P_ODR_50HZ;
    mcfg.osr   = QMC5883P_OSR_8;
    mcfg.dsr   = QMC5883P_DSR_1;
    mcfg.mode  = QMC5883P_MODE_CONTINUOUS;

    st = qmc5883p_init(&mag, &mag_i2c, &mcfg);
    if (st != QMC5883P_OK) {
        uint8_t id = 0U;

        print_code("QMC5883P_Init FAILED. code=", (int)st);
        if (st == QMC5883P_ERR_BUS) {
            print_code("  i2c status=", (int)mag.last_i2c);
        }
        if (st == QMC5883P_ERR_ID && qmc5883p_read_id(&mag, &id) == QMC5883P_OK) {
            char line[LINE_BUF_SIZE];
            char *p = line;
            p = put_str(p, "  CHIP_ID read = 0x");
            p = put_hex8(p, id);
            p = put_str(p, " (driver expects 0x80)\r\n");
            console_write(line, (uint16_t)(p - line));
        }
        for (;;) { delay_ms(1000U); }
    }
    console_print("QMC5883P initialization successful.\r\n");
    dump_regs();

    qmc5883p_set_declination(&mag, DECLINATION_DEG);

#if DO_CALIBRATION
    console_print("\r\n[3] Calibration: ROTATE the board in all directions for 10 s...\r\n");
    st = qmc5883p_calibrate(&mag, CALIB_SAMPLES);
    if (st == QMC5883P_OK) {
        float off[3];
        float sc[3];
        char line[LINE_BUF_SIZE];
        char *p = line;

        qmc5883p_get_calibration(&mag, off, sc);
        p = put_str(p, "Offset: ");
        p = put_fixed3(p, off[0]);
        p = put_str(p, " ");
        p = put_fixed3(p, off[1]);
        p = put_str(p, " ");
        p = put_fixed3(p, off[2]);
        p = put_str(p, "\r\nScale : ");
        p = put_fixed3(p, sc[0]);
        p = put_str(p, " ");
        p = put_fixed3(p, sc[1]);
        p = put_str(p, " ");
        p = put_fixed3(p, sc[2]);
        p = put_str(p, "\r\n");
        console_write(line, (uint16_t)(p - line));
    } else {
        print_code("Calibration failed. code=", (int)st);
    }
#endif

    console_print("\r\n--- LIVE MAG DATA (50 Hz) ---\r\n");

    /* Step 5: Continuous output (read blocks until a new sample is ready) */
    for (;;) {
        st = qmc5883p_read(&mag, &data);

        if (st == QMC5883P_OK) {
            char line[LINE_BUF_SIZE];
            char *p = line;

            p = put_str(p, "X: ");
            p = put_fixed3(p, data.x);
            p = put_str(p, " Y: ");
            p = put_fixed3(p, data.y);
            p = put_str(p, " Z: ");
            p = put_fixed3(p, data.z);
            p = put_str(p, " Ga | |B|: ");
            p = put_fixed3(p, data.magnitude_g);
            p = put_str(p, " | Heading: ");
            p = put_fixed3(p, data.heading_deg);
            p = put_str(p, " deg\r\n");
            console_write(line, (uint16_t)(p - line));
        } else if (st == QMC5883P_ERR_OVERFLOW) {
            console_print("Sensor overflow (field too strong - use a larger range)\r\n");
        } else {
            print_code("QMC5883P read failed. code=", (int)st);
            if (st == QMC5883P_ERR_BUS) {
                print_code("  i2c status=", (int)mag.last_i2c);
            }
            delay_ms(100U);
        }
    }
}