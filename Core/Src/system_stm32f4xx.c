#include "stm32f4xx.h"
#include <stdint.h>

uint32_t SystemCoreClock = 16000000; /* Default 16 MHz HSI Clock */
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
  /* Enable FPU (CP10 and CP11 full access) */
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
  #endif

  /* Reset Clock Control Register */
  RCC->CR |= (uint32_t)0x00000001;
  RCC->CFGR = 0x00000000;
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  RCC->PLLCFGR = 0x24003010;
  RCC->CR &= (uint32_t)0xFFFAFFFF;
  RCC->CIR = 0x00000000;

  /* Set Vector Table Location */
  #ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE;
  #else
    SCB->VTOR = FLASH_BASE;
  #endif
}

void SystemCoreClockUpdate(void)
{
  SystemCoreClock = 16000000;
}