/**
 * @file    gps_bus_stm32.c
 * @brief   Bare-metal (zero HAL_*) implementation of the GPS_Bus contract
 *          for STM32F411 hardware, built directly on
 *          Core/Src/Protocols/UART/uart_bare.c.
 *
 * @details Wiring assumption on STM32F411CEU6 Black Pill:
 *          GPS/GNSS module (u-blox NEO-M8N) on USART2:
 *            - PA2 = USART2_TX (AF7, Push-Pull, Very High Speed)
 *            - PA3 = USART2_RX (AF7, Pull-up enabled to prevent line float)
 *          Leaves USART1 free for debug telemetry / console.
 */

#include "gps_bus.h"
#include "uart_bare.h"
#include <stddef.h>

/*============================================================================*
 *                   STM32F411 PERIPHERAL BASE ADDRESSES (RM0383)             *
 *============================================================================*/

#define PERIPH_BASE_ADDR        0x40000000UL
#define AHB1PERIPH_BASE_ADDR    (PERIPH_BASE_ADDR + 0x00020000UL)
#define APB1PERIPH_BASE_ADDR    (PERIPH_BASE_ADDR + 0x00000000UL)

#define RCC_BASE_ADDR           (AHB1PERIPH_BASE_ADDR + 0x3800UL)
#define GPIOA_BASE_ADDR         (AHB1PERIPH_BASE_ADDR + 0x0000UL)
#define USART2_BASE_ADDR        (APB1PERIPH_BASE_ADDR + 0x4400UL)

/* RCC Enable Registers */
#define RCC_AHB1ENR_REG         (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x30UL))
#define RCC_APB1ENR_REG         (*(volatile uint32_t *)(RCC_BASE_ADDR + 0x40UL))

#define RCC_AHB1ENR_GPIOAEN_BIT (1UL << 0)
#define RCC_APB1ENR_USART2EN_BIT (1UL << 17)

/* GPIOA Registers (RM0383 §8.4) */
#define GPIOA_MODER_REG         (*(volatile uint32_t *)(GPIOA_BASE_ADDR + 0x00UL))
#define GPIOA_OTYPER_REG        (*(volatile uint32_t *)(GPIOA_BASE_ADDR + 0x04UL))
#define GPIOA_OSPEEDR_REG       (*(volatile uint32_t *)(GPIOA_BASE_ADDR + 0x08UL))
#define GPIOA_PUPDR_REG         (*(volatile uint32_t *)(GPIOA_BASE_ADDR + 0x0CUL))
#define GPIOA_AFRL_REG          (*(volatile uint32_t *)(GPIOA_BASE_ADDR + 0x20UL))

#define PIN_USART2_TX_NUM       2U
#define PIN_USART2_RX_NUM       3U
#define GPIO_AF7_USART2         7U

/** @brief Single UART handle for GPS on USART2 */
static UART_Bare_Handle_t s_gps_uart;

/**
 * @brief One-time USART2 + GPIOA bring-up for STM32F411. Call before GPS_Init().
 * @param peripheral_clock_hz APB1 clock actually feeding USART2 (e.g. 50000000UL)
 * @param baud_rate           Module's configured baud rate (9600 default for NEO-M8N)
 */
void gps_bus_stm32_setup(uint32_t peripheral_clock_hz, uint32_t baud_rate)
{
    /* 1. Enable RCC clocks for GPIOA (AHB1) and USART2 (APB1) */
    RCC_AHB1ENR_REG |= RCC_AHB1ENR_GPIOAEN_BIT;
    RCC_APB1ENR_REG |= RCC_APB1ENR_USART2EN_BIT;

    /* 2. Configure PA2 and PA3 as Alternate Function (MODER bits = 0b10) */
    GPIOA_MODER_REG &= ~((3UL << (PIN_USART2_TX_NUM * 2U)) | (3UL << (PIN_USART2_RX_NUM * 2U)));
    GPIOA_MODER_REG |=  ((2UL << (PIN_USART2_TX_NUM * 2U)) | (2UL << (PIN_USART2_RX_NUM * 2U)));

    /* 3. Configure output type as Push-Pull (OTYPER bit = 0) */
    GPIOA_OTYPER_REG &= ~((1UL << PIN_USART2_TX_NUM) | (1UL << PIN_USART2_RX_NUM));

    /* 4. Configure output speed as Very High Speed (OSPEEDR bits = 0b11) */
    GPIOA_OSPEEDR_REG &= ~((3UL << (PIN_USART2_TX_NUM * 2U)) | (3UL << (PIN_USART2_RX_NUM * 2U)));
    GPIOA_OSPEEDR_REG |=  ((3UL << (PIN_USART2_TX_NUM * 2U)) | (3UL << (PIN_USART2_RX_NUM * 2U)));

    /* 5. Configure pull-up on RX (PA3) to prevent floating line when disconnected; no pull on TX (PA2) */
    GPIOA_PUPDR_REG &= ~((3UL << (PIN_USART2_TX_NUM * 2U)) | (3UL << (PIN_USART2_RX_NUM * 2U)));
    GPIOA_PUPDR_REG |=  (1UL << (PIN_USART2_RX_NUM * 2U));

    /* 6. Configure AF7 (USART2) in AFRL (AFR[0]) for PA2 and PA3 */
    GPIOA_AFRL_REG &= ~((0xFUL << (PIN_USART2_TX_NUM * 4U)) | (0xFUL << (PIN_USART2_RX_NUM * 4U)));
    GPIOA_AFRL_REG |=  (((uint32_t)GPIO_AF7_USART2 << (PIN_USART2_TX_NUM * 4U)) |
                        ((uint32_t)GPIO_AF7_USART2 << (PIN_USART2_RX_NUM * 4U)));

    /* 7. Initialize UART Bare peripheral driver */
    UART_Bare_Config_t config = {
        .peripheral_clock_hz = peripheral_clock_hz,
        .baud_rate           = baud_rate,
        .timeout_iters       = 1000000U,
    };
    (void)UART_Bare_Init(&s_gps_uart, (volatile uint32_t *)USART2_BASE_ADDR, &config);
}

/*============================================================================*
 *                     GPS_Bus CONTRACT IMPLEMENTATION                        *
 *============================================================================*/

int GPS_BusReadByte(void *bus_context, uint8_t *byte_out)
{
    (void)bus_context;
    UART_Bare_Status_t st = UART_Bare_ReceiveByte(&s_gps_uart, byte_out);
    switch (st) {
        case UART_BARE_OK:            return 0;
        case UART_BARE_ERROR_NO_DATA: return 1;
        default:                      return -1;
    }
}

int GPS_BusWrite(void *bus_context, const uint8_t *data, uint16_t length)
{
    (void)bus_context;
    if (data == NULL && length > 0U) {
        return -1;
    }
    UART_Bare_Status_t st = UART_Bare_Transmit(&s_gps_uart, data, length);
    return (st == UART_BARE_OK) ? 0 : -1;
}

void GPS_DelayMs(uint32_t ms)
{
    /* Calibrated spin-delay loop for STM32F411 at 100 MHz SYSCLK (~20,000 iters per ms) */
    for (volatile uint32_t i = 0; i < ms; i++) {
        for (volatile uint32_t j = 0; j < 20000U; j++) {
            __asm__("nop");
        }
    }
}
