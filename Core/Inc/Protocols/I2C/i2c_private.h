/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    I2C_private.h    >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Alaa Hassan                  **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : STM32F411CEU6 (BlackPill)                       **
 **                  SWC    : I2C                                             **
 **                                                                           **
 **===========================================================================**
 */
#ifndef _I2C_PRIVATE_H_
#define _I2C_PRIVATE_H_

/* Number of I2C peripherals available on STM32F411CEU6 (I2C1, I2C2, I2C3) */
#define I2C_CHANNEL_COUNT   3

/************************** I2C_CR1 Bit Definitions **************************/
typedef enum
{
  CR1_PE = 0,      /* Peripheral Enable                 */
  CR1_SMBUS = 1,   /* SMBus mode                        */
  CR1_SMBTYPE = 3, /* SMBus type                        */
  CR1_ENARP = 4,   /* ARP enable                        */
  CR1_ENPEC = 5,   /* PEC enable                        */
  CR1_ENGC = 6,    /* General call enable               */
  CR1_NOSTRETCH = 7, /* Clock stretching disable (slave)*/
  CR1_START = 8,   /* Start generation                  */
  CR1_STOP = 9,    /* Stop generation                   */
  CR1_ACK = 10,    /* Acknowledge enable                */
  CR1_POS = 11,    /* ACK/PEC position                  */
  CR1_PEC = 12,    /* Packet error checking             */
  CR1_ALERT = 13,  /* SMBus alert                       */
  CR1_SWRST = 15,  /* Software reset                    */
} I2C_CR1_BITS;

/************************** I2C_CR2 Bit Definitions **************************/
typedef enum
{
  CR2_FREQ0 = 0,   /* Peripheral clock frequency [5:0] starts here */
  CR2_ITERREN = 8, /* Error interrupt enable            */
  CR2_ITEVTEN = 9, /* Event interrupt enable            */
  CR2_ITBUFEN = 10,/* Buffer interrupt enable           */
  CR2_DMAEN = 11,  /* DMA requests enable               */
  CR2_LAST = 12,   /* DMA last transfer                 */
} I2C_CR2_BITS;

#define CR2_FREQ_MASK  (0x3F) /* 6-bit field */

/************************** I2C_OAR1 Bit Definitions **************************/
typedef enum
{
  OAR1_ADD0 = 0,   /* bit 0 of the address (10-bit mode only) */
  OAR1_ADD1_7 = 1, /* bits [7:1] of the address                */
  OAR1_ADD8_9 = 8, /* bits [9:8] of the address (10-bit mode)  */
  OAR1_MUST_BE_1 = 14, /* must always be kept at 1 per RM     */
  OAR1_ADDMODE = 15,   /* addressing mode (7-bit/10-bit)      */
} I2C_OAR1_BITS;

/************************** I2C_SR1 Bit Definitions **************************/
typedef enum
{
  SR1_SB = 0,      /* Start bit generated                */
  SR1_ADDR = 1,    /* Address sent/matched               */
  SR1_BTF = 2,     /* Byte transfer finished              */
  SR1_ADD10 = 3,   /* 10-bit header sent                  */
  SR1_STOPF = 4,   /* Stop detection (slave)              */
  SR1_RXNE = 6,    /* Data register not empty (receiver)  */
  SR1_TXE = 7,     /* Data register empty (transmitter)   */
  SR1_BERR = 8,    /* Bus error                           */
  SR1_ARLO = 9,    /* Arbitration lost                    */
  SR1_AF = 10,     /* Acknowledge failure                 */
  SR1_OVR = 11,    /* Overrun/underrun                    */
  SR1_PECERR = 12, /* PEC error in reception               */
  SR1_TIMEOUT = 14,/* Timeout / Tlow error                */
  SR1_SMBALERT = 15,/* SMBus alert                        */
} I2C_SR1_BITS;

/************************** I2C_SR2 Bit Definitions **************************/
typedef enum
{
  SR2_MSL = 0,     /* Master/Slave                        */
  SR2_BUSY = 1,    /* Bus busy                            */
  SR2_TRA = 2,     /* Transmitter/Receiver                */
  SR2_GENCALL = 4, /* General call address (slave mode)   */
  SR2_SMBDEFAULT = 5,
  SR2_SMBHOST = 6,
  SR2_DUALF = 7,   /* Dual flag (slave mode)               */
} I2C_SR2_BITS;

/************************** I2C_CCR Bit Definitions **************************/
#define CCR_CCR_MASK   (0xFFF) /* bits [11:0] */
#define CCR_DUTY       (14)
#define CCR_FS         (15)

/************************** I2C_TRISE Bit Definitions **************************/
#define TRISE_MASK     (0x3F) /* bits [5:0] */

#define I2C_u32TIMEOUT  10000UL

#endif /* _I2C_PRIVATE_H_ */
