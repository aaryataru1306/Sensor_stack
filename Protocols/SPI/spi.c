#include "spi.h"
#include "uart.h"

extern UART_Handle_t g_debug_uart; /* Declared and initialized in main.c */

void SPI_CS_Init(uint8_t pinNumber) {
    /* 1. Enable GPIOA Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* 2. Enable Internal Pull-Up */
    GPIOA->PUPDR &= ~(3U << (pinNumber * 2U));
    GPIOA->PUPDR |=  (1U << (pinNumber * 2U));

    /* 3. Pre-load BSRR HIGH before changing mode to Output (Glitch-Free) */
    GPIOA->BSRR = (1U << pinNumber);

    /* 4. Set Pin Mode to Output (01) */
    GPIOA->MODER &= ~(3U << (pinNumber * 2U));
    GPIOA->MODER |=  (1U << (pinNumber * 2U));
}

void SPI_CS_Select(uint8_t pinNumber) {
    GPIOA->BSRR = (1U << (pinNumber + 16U)); /* Drive LOW */
}

void SPI_CS_Deselect(uint8_t pinNumber) {
    GPIOA->BSRR = (1U << pinNumber); /* Drive HIGH */
}

void SPI1_Init(void) {
    UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] Initializing SPI1 Master peripheral...\r\n", 
                  sizeof("[SPI1] Initializing SPI1 Master peripheral...\r\n") - 1U);

    /* 1. Enable GPIOA and SPI1 Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* 2. Configure SCK, MISO, MOSI as Alternate Function 5 (AF5) */
    GPIOA->MODER &= ~(  (3U << (SPI1_SCK_PIN  * 2U))
                      | (3U << (SPI1_MISO_PIN * 2U))
                      | (3U << (SPI1_MOSI_PIN * 2U)) );
    GPIOA->MODER |=  (  (2U << (SPI1_SCK_PIN  * 2U))
                      | (2U << (SPI1_MISO_PIN * 2U))
                      | (2U << (SPI1_MOSI_PIN * 2U)) );

    GPIOA->AFR[0] &= ~(  (0xFU << (SPI1_SCK_PIN  * 4U))
                       | (0xFU << (SPI1_MISO_PIN * 4U))
                       | (0xFU << (SPI1_MOSI_PIN * 4U)) );
    GPIOA->AFR[0] |=  (  (0x5U << (SPI1_SCK_PIN  * 4U))
                       | (0x5U << (SPI1_MISO_PIN * 4U))
                       | (0x5U << (SPI1_MOSI_PIN * 4U)) );

    /* Set Very High Speed for SCK, MISO, and MOSI */
    GPIOA->OSPEEDR |= (  (3U << (SPI1_SCK_PIN  * 2U))
                       | (3U << (SPI1_MISO_PIN * 2U))
                       | (3U << (SPI1_MOSI_PIN * 2U)) );

    /* 3. Initialize Glitch-Free CS Pins for default sensors */
    SPI_CS_Init(SPI1_CS_ICM20948_PIN);
    SPI_CS_Init(SPI1_CS_BMP388_PIN);

    /* 4. Configure SPI1 CR1: Master, Software NSS (SSI/SSM), Prescaler DIV16, SPE=1 */
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (5U << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;

    UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] Init Complete (6.25 MHz, Master Mode 0, CS PA1 & PA4 Ready)\r\n", 
                  sizeof("[SPI1] Init Complete (6.25 MHz, Master Mode 0, CS PA1 & PA4 Ready)\r\n") - 1U);
}

uint8_t SPI1_TransferByte(uint8_t data) {
    uint32_t timeout = 10000U;

    while (!(SPI1->SR & SPI_SR_TXE) && --timeout);
    if (timeout == 0U) {
        UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] ERROR: TXE Timeout!\r\n", 
                      sizeof("[SPI1] ERROR: TXE Timeout!\r\n") - 1U);
        return 0xFF;
    }

    SPI1->DR = data;

    timeout = 10000U;
    while (!(SPI1->SR & SPI_SR_RXNE) && --timeout);
    if (timeout == 0U) {
        UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] ERROR: RXNE Timeout!\r\n", 
                      sizeof("[SPI1] ERROR: RXNE Timeout!\r\n") - 1U);
        return 0xFF;
    }

    uint8_t rxData = (uint8_t)SPI1->DR;

    /* Wait for BSY bit to clear before returning so CS isn't cut off early */
    timeout = 10000U;
    while ((SPI1->SR & SPI_SR_BSY) && --timeout);
    if (timeout == 0U) {
        UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] ERROR: BSY Timeout!\r\n", 
                      sizeof("[SPI1] ERROR: BSY Timeout!\r\n") - 1U);
    }

    return rxData;
}

void SPI1_ReadRegisters(uint8_t cs_pin, uint8_t reg_addr, uint8_t *buffer, uint16_t length) {
    if (buffer == 0 && length > 0U) {
        UART_Transmit(&g_debug_uart, (const uint8_t *)"[SPI1] ERROR: Null Buffer Passed!\r\n", 
                      sizeof("[SPI1] ERROR: Null Buffer Passed!\r\n") - 1U);
        return;
    }

    SPI_CS_Select(cs_pin);
    (void)SPI1_TransferByte(reg_addr);

    for (uint16_t i = 0U; i < length; i++) {
        buffer[i] = SPI1_TransferByte(0xFFU);
    }

    SPI_CS_Deselect(cs_pin);
}

void SPI1_WriteRegister(uint8_t cs_pin, uint8_t reg_addr, uint8_t value) {
    SPI_CS_Select(cs_pin);
    (void)SPI1_TransferByte(reg_addr);
    (void)SPI1_TransferByte(value);
    SPI_CS_Deselect(cs_pin);
}