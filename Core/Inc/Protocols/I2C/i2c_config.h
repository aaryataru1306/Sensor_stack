/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    I2C_config.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Alaa Hassan                                     **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : STM32F411CEU6 (BlackPill)                       **
 **                  SWC    : I2C                                             **
 **                                                                           **
 **===========================================================================**
 */
#ifndef _I2C_CONFIG_H_
#define _I2C_CONFIG_H_

/* This file is used for user configurations.                               */
/* Example: pre-built configuration objects can be declared here and        */
/* referenced from the application, the same way SPI_config.h is used.      */
/*                                                                          */
/* Example (uncomment and adjust to your bus clock):                        */
/*                                                                          */
/* I2C_Config_t I2C1_Config =                                               */
/* {                                                                        */
/*     .Channel       = I2C_CHANNEL1,                                       */
/*     .Mode          = I2C_MODE_I2C,                                       */
/*     .ACK           = I2C_ACK_ENABLE,                                     */
/*     .AddressMode   = I2C_ADDR_MODE_7BIT,                                 */
/*     .OwnAddress    = 0x00,                                               */
/*     .Speed         = I2C_SPEED_FAST,                                     */
/*     .FM_DutyCycle  = I2C_DUTY_2,                                         */
/*     .ClockSpeed_Hz = 400000UL,                                           */
/*     .PCLK1_Hz      = 16000000UL,                                         */
/* };                                                                       */

#endif /* _I2C_CONFIG_H_ */
