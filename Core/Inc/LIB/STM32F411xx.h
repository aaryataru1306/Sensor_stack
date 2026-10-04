/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    STM32F446xx.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Alaa Hassan                                     **
 **                  Layer  : LIB                                             **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : STM32F411CEU6 (BlackPill)                       **
 **                  SWC    : STM32F446xx                                     **
 **                                                                           **
 **  NOTE: This file was originally written for STM32F446RE (Nucleo-64) and  **
 **        is reused here for STM32F411CEU6. The only functional addition   **
 **        needed for the I2C driver is the I2C register map + base         **
 **        addresses below (I2C was not defined in the original file).      **
 **        See the porting notes at the bottom of this file for the other   **
 **        differences between F446 and F411 that affect the other MCAL     **
 **        drivers (GPIO ports, SPI channels, NVIC IRQ table).               **
 **===========================================================================**
 */

#ifndef STM32F446xx_H
#define STM32F446xx_H

/**************************************         Various Memories Base Adresses          ******************************************/
#define FLASH_BASEADDR  0x08000000UL
#define SRAM_BASEADDR   0x20000000UL
#define ROM_BASEADDR    0x1FFF0000UL

/**************************************         NVIC Base Adresses          ******************************************/
#define NVIC_BASEADDR   0XE000E100UL

/**************************************         SCB Base Adresses          ******************************************/
#define SCB_BASEADDR    0XE000ED00UL

/**************************************         AHB1 Peripheral Base Adresses          ******************************************/
#define GPIOA_BASEADDR  0X40020000UL
#define GPIOB_BASEADDR  0X40020400UL
#define GPIOC_BASEADDR  0X40020800UL
#define GPIOD_BASEADDR  0X40020C00UL
#define GPIOE_BASEADDR  0X40021000UL
#define GPIOF_BASEADDR  0X40021400UL
#define GPIOG_BASEADDR  0X40021800UL
#define GPIOH_BASEADDR  0X40021C00UL
/* NOTE: STM32F411CEU6 physically implements GPIOA, B, C, D, E and H only.
 *       GPIOF_BASEADDR/GPIOG_BASEADDR are kept here for source compatibility
 *       with the F446 driver files, but must NOT be used on the F411 (see
 *       porting notes at the end of this file). */

#define RCC_BASEADDR    0x40023800UL

#define SYSTIC_BASEADDR 0XE000E010UL

/*Internal DMA Base Adresses */
#define DMA1_BASEADDR   0X40026000UL
#define DMA2_BASEADDR   0X40026400UL

/**************************************         AHB2 Peripheral Base Adresses          ******************************************/
/**************************************         AHB3 Peripheral Base Adresses          ******************************************/
/**************************************         APB1 Peripheral Base Adresses          ******************************************/

#define USART2_BASEADDR 0x40004400UL
#define USART3_BASEADDR 0x40004800UL
#define USART4_BASEADDR 0x40004C00UL
#define USART5_BASEADDR 0x40005000UL
#define SPI2_BASEADDR   0X40003800UL
#define SPI3_BASEADDR   0X40003C00UL

/* Added for the I2C driver: I2C1/2/3 all live on APB1 on the whole F4 family */
#define I2C1_BASEADDR   0x40005400UL
#define I2C2_BASEADDR   0x40005800UL
#define I2C3_BASEADDR   0x40005C00UL

/**************************************         APB2 Peripheral Base Adresses          ******************************************/

#define SYSCFG_BASEADDR 0X40013800UL
#define EXTI_BASEADDR   0X40013C00UL
#define USART1_BASEADDR 0X40011000UL
#define USART6_BASEADDR 0X40011400UL
#define SPI1_BASEADDR   0X40013000UL
#define SPI4_BASEADDR   0X40013400UL
#define SPI5_BASEADDR   0X40015000UL
/* NOTE: SPI5 is available on STM32F411 (and not on the F446 driver's original
 *       SPI_CHANNEL_COUNT of 4) - see porting notes if you need SPI5. */

/**************************************         APB3 Peripheral Base Adresses          ******************************************/

/**************************************         SYSTIC Peripheral Definitions       *********************************************/

typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile uint32_t CALIB;
} SYSTIC_RegDef_t;

#define MSYSTIC ((SYSTIC_RegDef_t *)SYSTIC_BASEADDR)

/**************************************       GPIO Register Definition Structure       ******************************************/
typedef struct
{
  volatile uint32_t MODER;   /* GPIO PORT mode register              */
  volatile uint32_t OTYPER;  /* GPIO PORT output type register       */
  volatile uint32_t OSPEEDR; /* GPIO PORT output speed register      */
  volatile uint32_t PUPDR;   /* GPIO PORT pull-up/pull-down register */
  volatile uint32_t IDR;     /* GPIO PORT input data register        */
  volatile uint32_t ODR;     /* GPIO PORT output data register       */
  volatile uint32_t BSRR;    /* GPIO PORT bit set/reset register     */
  volatile uint32_t LCKR;    /* GPIO PORT configuration lock register*/
  volatile uint32_t AFR[2];  /* GPIO alternate function low register */
} GPIO_REGDEF_t;


/**************************************       DMA Regster Definitions Structure       ******************************************/
typedef struct
{
  uint32_t CR;       // Stream x configuration register (DMA_SxCR)
  uint32_t NDTR;     // Stream x number of data register (DMA_SxNDTR)
  uint32_t PAR;      // Stream x peripheral address register (DMA_SxPAR)
  uint32_t M0AR;     // Stream x memory 0 address register (DMA_SxM0AR)
  uint32_t M1AR;     // Stream x memory 1 address register (DMA_SxM1AR)
  uint32_t FCR;      // Stream x FIFO control register (DMA_SxFCR)
}DMA_STREAM_REGDEF_t;

typedef struct
{
  uint32_t LISR;      // Low interrupt status register (DMA_LISR)
  uint32_t HISR;      // High interrupt status register (DMA_HISR)
  uint32_t LIFCR;     // Low interrupt flag clear register (DMA_LIFCR)
  uint32_t HIFCR;     // High interrupt flag clear register (DMA_HIFCR)
  DMA_STREAM_REGDEF_t Stream[8];
}DMA_REGDEF_t;

#define MDMA1 ((DMA_REGDEF_t *)DMA1_BASEADDR)
#define MDMA2 ((DMA_REGDEF_t *)DMA2_BASEADDR)

/**************************************       RCC Register Definitions Structure       ******************************************/
typedef struct
{
  volatile uint32_t CR;           /* RCC clock control register                                                 */
  volatile uint32_t PLLCFGR;      /* RCC PLL configuration register (RCC_PLLCFGR)                               */
  volatile uint32_t CFGR;         /* RCC clock configuration register (RCC_CFGR)                                */
  volatile uint32_t CIR;          /* RCC clock interrupt register (RCC_CIR)                                     */
  volatile uint32_t AHP1RSTR;     /* RCC AHB1 peripheral reset register (RCC_AHB1RSTR)                          */
  volatile uint32_t AHP2RSTR;     /* RCC AHB2 peripheral reset register (RCC_AHB2RSTR)                          */
  volatile uint32_t AHP3RSTR;     /* RCC AHB3 peripheral reset register (RCC_AHB3RSTR)                          */
  volatile uint32_t RESERVED1[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register (RCC_APB1RSTR)                          */
  volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register (RCC_APB2RSTR)                          */
  volatile uint32_t RESERVED2[2]; /* RESERVED                                                                   */
  volatile uint32_t AHP1ENR;      /* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)                    */
  volatile uint32_t AHP2ENR;      /* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)                    */
  volatile uint32_t AHP3ENR;      /* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)                    */
  volatile uint32_t RESERVED3[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register (RCC_APB1ENR)                    */
  volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register (RCC_APB2ENR)                    */
  volatile uint32_t RESERVED4[2]; /* RESERVED                                                                   */
  volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register(RCC_AHB1LPENR) */
  volatile uint32_t AHP2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode register(RCC_AHB2LPENR) */
  volatile uint32_t AHP3LPENR;    /* RCC AHB3 peripheral clock enable in low power mode register(RCC_AHB3LPENR) */
  volatile uint32_t RESERVED5[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register(RCC_APB1LPENR) */
  volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enabled in low power mode register(RCC_APB2LPENR)*/
  volatile uint32_t RESERVED6[2]; /* RESERVED                                                                   */
  volatile uint32_t BDCR;         /* RCC Backup domain control register (RCC_BDCR)                              */
  volatile uint32_t CSR;          /* RCC clock control & status register (RCC_CSR)                              */
  volatile uint32_t RESERVED7[2]; /* RESERVED                                                                   */
  volatile uint32_t SSCGR;        /* RCC spread spectrum clock generation register (RCC_SSCGR)                  */
  volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register (RCC_PLLI2SCFGR)                         */
  volatile uint32_t PLLSAICFGR;   /* RCC PLL configuration register (RCC_PLLSAICFGR)                            */
  volatile uint32_t DCKCFGR;      /* RCC dedicated clock configuration register (RCC_DCKCFGR)                   */
  volatile uint32_t CKGATENR;     /* RCC clocks gated enable register (CKGATENR)                                */
  volatile uint32_t DCKCFGR2;     /* RCC dedicated clocks configuration register 2 (DCKCFGR2)                   */
} RCC_RegDef_t;

/**************************************       SPI Register Definitions Structure       ******************************************/
typedef struct
{
  volatile uint32_t CR1;                /* SPI Control Register 1 */
  volatile uint32_t CR2;                /* SPI Control Register 2 */
  volatile uint32_t SR;                 /* SPI Status Register */
  volatile uint32_t DR;                 /* SPI Data Register */
  volatile uint32_t CRCPR;              /* SPI CRC Polynomial Register */
  volatile uint32_t RXCRCR;             /* SPI Rx CRC Register */
  volatile uint32_t TXCRCR;             /* SPI Tx CRC Register */
  volatile uint32_t I2SCFGR;            /* SPI I2S Configuration Register */
  volatile uint32_t I2SPR;              /* SPI I2S Prescaler Register */
} SPI_RegDef_t;

/**************************************         SPI Peripheral Definitions       *********************************************/
#define MSPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define MSPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define MSPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define MSPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)
#define MSPI5 ((SPI_RegDef_t *)SPI5_BASEADDR)

/**************************************       I2C Register Definitions Structure       ******************************************/
/* Added for STM32F411 I2C driver - register layout is identical across the whole STM32F4 family */
typedef struct
{
  volatile uint32_t CR1;    /* I2C Control register 1        */
  volatile uint32_t CR2;    /* I2C Control register 2        */
  volatile uint32_t OAR1;   /* I2C Own address register 1    */
  volatile uint32_t OAR2;   /* I2C Own address register 2    */
  volatile uint32_t DR;     /* I2C Data register              */
  volatile uint32_t SR1;    /* I2C Status register 1          */
  volatile uint32_t SR2;    /* I2C Status register 2          */
  volatile uint32_t CCR;    /* I2C Clock control register     */
  volatile uint32_t TRISE;  /* I2C TRISE register              */
  volatile uint32_t FLTR;   /* I2C FLTR register (noise filter)*/
} I2C_RegDef_t;

/**************************************         I2C Peripheral Definitions       *********************************************/
#define MI2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define MI2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define MI2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

/**************************************         NVIC Peripheral Definitions       *********************************************/
typedef struct
{
  volatile uint32_t ISER[8];        /* Interrupt Set Enable Register */
  volatile uint32_t RESERVED1[24];
  volatile uint32_t ICER[8];        /* Interrupt Clear Enable Register */
  volatile uint32_t RESERVED2[24];
  volatile uint32_t ISPR[8];        /* Interrupt Set Pending Register */
  volatile uint32_t RESERVED3[24];
  volatile uint32_t ICPR[8];        /* Interrupt Clear Pending Register */
  volatile uint32_t RESERVED4[24];
  volatile uint32_t IABR[8];        /* Interrupt Active Bit Register */
  volatile uint32_t RESERVED5[56];
  volatile uint8_t  IPR[240];        /* Interrupt Priority Register */
  volatile uint32_t RESERVED6[580];
  volatile uint32_t STIR;           /* Software Trigger Interrupt Register */
} NVIC_RegDef_t;

#define MNVIC ((NVIC_RegDef_t *)NVIC_BASEADDR)

/**************************************         GPIO Peripheral Definitions       ******************************************/

#define MGPIOA ((GPIO_REGDEF_t *)GPIOA_BASEADDR)
#define MGPIOB ((GPIO_REGDEF_t *)GPIOB_BASEADDR)
#define MGPIOC ((GPIO_REGDEF_t *)GPIOC_BASEADDR)
#define MGPIOD ((GPIO_REGDEF_t *)GPIOD_BASEADDR)
#define MGPIOE ((GPIO_REGDEF_t *)GPIOE_BASEADDR)
#define MGPIOF ((GPIO_REGDEF_t *)GPIOF_BASEADDR)
#define MGPIOG ((GPIO_REGDEF_t *)GPIOG_BASEADDR)
#define MGPIOH ((GPIO_REGDEF_t *)GPIOH_BASEADDR)
/* NOTE: on STM32F411CEU6 only MGPIOA, MGPIOB, MGPIOC, MGPIOD, MGPIOE and
 *       MGPIOH are backed by real silicon. Do not enable RCC clocks for, or
 *       access registers of, GPIOF/GPIOG on this MCU. */

/**************************************         RCC Peripheral Definitions       *********************************************/

#define MRCC ((RCC_RegDef_t *)RCC_BASEADDR)

/**************************************         SCB Peripheral Definitions       *********************************************/

typedef struct
{
  uint32_t CPUID;     // CPU Identification Register
  uint32_t ICSR;      // Interrupt Control and State Register
  uint32_t VTOR;      // Vector Table Offset Register
  uint32_t AIRCR;     // Application Interrupt and Reset Control Register
  uint32_t SCR;       // System Control Register
  uint32_t CCR;       // Configuration and Control Register
  uint32_t SHPR1;     // System Handler Priority Register 1 (Priority of SVCall)
  uint32_t SHPR2;     // System Handler Priority Register 2 (Priority of Debug Monitor)
  uint32_t SHPR3;     // System Handler Priority Register 3 (Priority of PendSV and SysTick)
  uint32_t SHCSR;     // System Handler Control and State Register
  uint8_t  CFSR;      // Configurable Fault Status Register (lower 8 bits)
  uint8_t  BFSR;      // BusFault Status Register (lower 8 bits)
  uint16_t UFSR;      // UsageFault Status Register (lower 16 bits)
  uint32_t HFSR;      // HardFault Status Register
  uint32_t DFSR;      // Debug Fault Status Register
  uint32_t MMAR;      // MemManage Fault Address Register
  uint32_t BFAR;      // BusFault Address Register
  uint32_t AFSR;      // Auxiliary Fault Status Register
} SCB_RegDef_t;

#define MSCB ((SCB_RegDef_t *)SCB_BASEADDR)


/**************************************         SYSCFG Peripheral Definitions       *********************************************/

typedef struct
{
  uint32_t MEMRMP;    // Memory Remap Register
  uint32_t PMC;       // PMC Register
  uint32_t EXTICR[4]; // External Interrupt Configuration Register
  uint32_t Reserved1[2];
  uint32_t CMPCR;     // Compensation Cell Control Register
  uint32_t Reserved2[2];
  uint32_t CFGR;
} SYSCFG_RegDef_t;

#define MSYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

/**************************************         EXTI Peripheral Definitions       *********************************************/
typedef struct
{
  uint32_t IMR;       // Interrupt Mask Register
  uint32_t EMR;       // Event Mask Register
  uint32_t RTSR;      // Rising Trigger Selection Register
  uint32_t FTSR;      // Falling Trigger Selection Register
  uint32_t SWIER;     // Software Interrupt Event Register
  uint32_t PR;        // Pending Register
} EXTI_RegDef_t;

#define MEXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

/**************************************      USART Register Definitions Structure   ******************************************/

typedef struct
{
  uint32_t SR;
  uint32_t DR;
  uint32_t BRR;
  uint32_t CR1;
  uint32_t CR2;
  uint32_t CR3;
  uint32_t GTPR;
} USART_RegDef_t;

/**************************************         USART Peripheral Definitions       *********************************************/

#define MUSART1 ((USART_RegDef_t *)USART1_BASEADDR)
#define MUSART2 ((USART_RegDef_t *)USART2_BASEADDR)
#define MUSART3 ((USART_RegDef_t *)USART3_BASEADDR)
#define MUSART4 ((USART_RegDef_t *)USART4_BASEADDR)
#define MUSART5 ((USART_RegDef_t *)USART5_BASEADDR)
#define MUSART6 ((USART_RegDef_t *)USART6_BASEADDR)

#endif /* STM32F446xx_H */

/**
 **===========================================================================**
 ** PORTING NOTES: STM32F446RE (Nucleo)  --->  STM32F411CEU6 (BlackPill)      **
 **===========================================================================**
 *
 * 1) GPIO ports (GPIO_private.h / GPIO_prog.c)
 *    F411 only has GPIOA, B, C, D, E and H (6 ports, no F/G).
 *    -> In GPIO_private.h change:  #define GPIO_PORT_COUNT 8u   to   6u
 *    -> In GPIO_prog.c change the array to:
 *         static GPIO_REGDEF_t *GPIO_Port[GPIO_PORT_COUNT] =
 *             {MGPIOA, MGPIOB, MGPIOC, MGPIOD, MGPIOE, MGPIOH};
 *       (GPIOF/GPIOG removed, GPIOH stays at the same enum index? NO -
 *        GPIO_Port_t enum in GPIO_interface.h must also drop GPIO_PORTF and
 *        GPIO_PORTG, and GPIO_PORTH becomes index 5 instead of 7.)
 *    -> In RCC_interface.h the RCC_GPIOFEN / RCC_GPIOGEN enum values simply
 *       must not be used (do not enable clocks for ports that don't exist).
 *
 * 2) SPI (SPI_private.h / SPI_program.c)
 *    F411 has 5 SPIs (SPI1..SPI5) instead of F446's 4.
 *    -> #define SPI_CHANNEL_COUNT 4  to  5
 *    -> static SPI_RegDef_t *SPI_Channel[SPI_CHANNEL_COUNT] =
 *           {MSPI1, MSPI2, MSPI3, MSPI4, MSPI5};
 *       (MSPI5 / SPI5_BASEADDR are already added above.)
 *
 * 3) I2C (this driver)
 *    Not present at all in the original F446 header -> I2C_RegDef_t,
 *    I2C1/2/3 base addresses and MI2C1/2/3 macros were added above.
 *
 * 4) NVIC (NVIC_interface.h)
 *    F411 does NOT have: CAN1/CAN2, SDIO, QUADSPI, FMPI2C1, SAI1/2, DCMI,
 *    OTG_HS. Do not enable/use these IRQ numbers on F411:
 *      NVIC_CAN1_TX, NVIC_CAN1_RX0, NVIC_CAN1_RX1, NVIC_CAN1_SCE,
 *      NVIC_SDIO, NVIC_QUADSPI, NVIC_FMPI2C1_EV, NVIC_FMPI2C1_ER.
 *    F411 DOES have SPI5 (NVIC_SPI5 is already present and valid) and I2C1/2/3
 *    event/error IRQs (NVIC_I2C1_EV/ER, NVIC_I2C2_EV/ER, NVIC_I2C3_EV/ER -
 *    already present in the enum) which you will need for an interrupt-driven
 *    I2C driver.
 *
 * 5) RCC (RCC_interface.h / RCC_program.c)
 *    Bus enable bit positions (AHB1ENR/APB1ENR/APB2ENR bit numbers) are
 *    identical across the F4 family, so RCC_interface.h/RCC_program.c need
 *    NO changes to use I2C1EN/I2C2EN/I2C3EN (already defined in
 *    RCC_APB1_BUS_t) or SPI5EN if you need it (not yet defined - add
 *    RCC_SPI5EN = 20 to RCC_APB2_BUS_t if required).
 *
 * 6) Clock speed assumption
 *    SYSTIC_config.h currently assumes SYSTEM_CLOCK_IN_MHZ = 16 (i.e. HSI,
 *    no PLL). The I2C_Config_t.PCLK1_Hz field you pass to I2C_enumInit()
 *    must always reflect your ACTUAL APB1 clock (get it from RCC/PLL
 *    configuration), not a hard-coded constant, otherwise the CCR/TRISE
 *    calculation - and therefore the SCL frequency - will be wrong.
 **===========================================================================**
 */
