/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    I2C_program.c    >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Alaa Hassan                                     **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : STM32F411CEU6 (BlackPill)                       **
 **                  SWC    : I2C                                             **
 **                                                                           **
 **===========================================================================**
 */
#include "stdint.h"
#include "ErrTypes.h"
#include "STM32F411xx.h" /* Register map for the target MCU: STM32F411CEU6 (was mistakenly STM32F446xx.h) */

#include "i2c_private.h"
#include "i2c_config.h"
#include "i2c_interface.h"

/* Array of I2C peripheral register definitions for easy access */
static I2C_RegDef_t *I2C_Channel[I2C_CHANNEL_COUNT] = {MI2C1, MI2C2, MI2C3};

/*=================================================================================================================*/
/**
 * @fn     I2C_vStop  (static helper)
 * @brief  Generates a STOP condition on the selected I2C bus
 */
static void I2C_vStop(I2C_Config_t *ChannelConfig)
{
  I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_STOP);
}

/*=================================================================================================================*/
/**
 * @fn     I2C_enumStart  (static helper)
 * @brief  Generates a START (or repeated START) condition and waits
 *         for the SB flag (Start Bit) to be set.
 * @return ErrorState_t: OK if successful, TIMEOUT_STATE otherwise
 */
static ErrorState_t I2C_enumStart(I2C_Config_t *ChannelConfig)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_START);

  while (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_SB) & 1) == 0 &&
         (Local_u32TimeoutCounter != I2C_u32TIMEOUT))
  {
    Local_u32TimeoutCounter++;
  }

  if (Local_u32TimeoutCounter == I2C_u32TIMEOUT)
  {
    Local_u8ErrorState = TIMEOUT_STATE;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     I2C_enumSendAddress  (static helper)
 * @brief  Sends the 7-bit slave address with the R/W bit and waits for
 *         the ADDR flag (address matched/ack received) to be set, or
 *         for an AF (acknowledge failure) flag in case the slave does
 *         not respond.
 * @return ErrorState_t: OK if the slave acknowledged, NOK if it did not,
 *         TIMEOUT_STATE if neither flag was set in time.
 */
static ErrorState_t I2C_enumSendAddress(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress, I2C_Direction_t Direction)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  I2C_Channel[ChannelConfig->Channel]->DR = (uint8_t)((SlaveAddress << 1) |
                                             ((Direction == I2C_DIRECTION_RECEIVER) ? 1U : 0U));

  while (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_ADDR) & 1) == 0 &&
         ((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_AF) & 1) == 0 &&
         (Local_u32TimeoutCounter != I2C_u32TIMEOUT))
  {
    Local_u32TimeoutCounter++;
  }

  if (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_AF) & 1) == 1)
  {
    /* Slave did not acknowledge its address: clear AF flag and report NOK */
    I2C_Channel[ChannelConfig->Channel]->SR1 &= ~(1 << SR1_AF);
    Local_u8ErrorState = NOK;
  }
  else if (Local_u32TimeoutCounter == I2C_u32TIMEOUT)
  {
    Local_u8ErrorState = TIMEOUT_STATE;
  }
  else
  {
    /* Clear ADDR flag with the required read of SR1 followed by SR2 */
    (void)I2C_Channel[ChannelConfig->Channel]->SR1;
    (void)I2C_Channel[ChannelConfig->Channel]->SR2;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     I2C_enumWriteByte  (static helper)
 * @brief  Waits for TXE (data register empty), writes one byte to DR,
 *         then waits for BTF (byte transfer finished) before returning.
 * @return ErrorState_t: OK if successful, TIMEOUT_STATE otherwise
 */
static ErrorState_t I2C_enumWriteByte(I2C_Config_t *ChannelConfig, uint8_t Data)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint32_t Local_u32TimeoutCounter = 0;

  while (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_TXE) & 1) == 0 &&
         (Local_u32TimeoutCounter != I2C_u32TIMEOUT))
  {
    Local_u32TimeoutCounter++;
  }

  if (Local_u32TimeoutCounter == I2C_u32TIMEOUT)
  {
    Local_u8ErrorState = TIMEOUT_STATE;
  }
  else
  {
    I2C_Channel[ChannelConfig->Channel]->DR = Data;

    Local_u32TimeoutCounter = 0;
    while (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_BTF) & 1) == 0 &&
           (Local_u32TimeoutCounter != I2C_u32TIMEOUT))
    {
      Local_u32TimeoutCounter++;
    }

    if (Local_u32TimeoutCounter == I2C_u32TIMEOUT)
    {
      Local_u8ErrorState = TIMEOUT_STATE;
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief Initialize the I2C peripheral with the provided configuration.
 *
 * This function configures the peripheral input clock (CR2.FREQ), the
 * bus clock generation (CCR/TRISE, derived from the requested
 * ClockSpeed_Hz and the supplied PCLK1_Hz), the own address (OAR1),
 * the I2C/SMBus mode, and finally enables the peripheral (PE) and the
 * acknowledge control (ACK).
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @return ErrorState_t OK if initialization successful, error code otherwise
 *
 * @warning NULL pointer check is performed on the input parameter.
 */
ErrorState_t I2C_enumInit(I2C_Config_t *ChannelConfig)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint16_t Local_u16CCR = 0;
  uint16_t Local_u16TRISE = 0;
  uint8_t  Local_u8FreqMHz = 0;

  if (ChannelConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else if (ChannelConfig->Channel >= I2C_CHANNEL_COUNT)
  {
    Local_u8ErrorState = NOK;
  }
  else
  {
    /* Disable the peripheral before (re)configuring it */
    I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_PE);

    /* Software reset pulse to guarantee a known state before configuration */
    I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_SWRST);
    I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_SWRST);

    /* Configure peripheral input clock frequency (APB1 clock in MHz) */
    Local_u8FreqMHz = (uint8_t)(ChannelConfig->PCLK1_Hz / 1000000UL);
    I2C_Channel[ChannelConfig->Channel]->CR2 &= ~CR2_FREQ_MASK;
    I2C_Channel[ChannelConfig->Channel]->CR2 |= (Local_u8FreqMHz & CR2_FREQ_MASK);

    /* Compute CCR and TRISE according to the requested speed */
    if (ChannelConfig->Speed == I2C_SPEED_STANDARD)
    {
      Local_u16CCR = (uint16_t)(ChannelConfig->PCLK1_Hz / (2UL * ChannelConfig->ClockSpeed_Hz));
      if (Local_u16CCR < 4U)
      {
        Local_u16CCR = 4U; /* minimum allowed CCR value in Standard mode */
      }
      Local_u16TRISE = (uint16_t)(Local_u8FreqMHz + 1U);

      I2C_Channel[ChannelConfig->Channel]->CCR &= ~(1 << CCR_FS);
    }
    else /* I2C_SPEED_FAST */
    {
      if (ChannelConfig->FM_DutyCycle == I2C_DUTY_2)
      {
        Local_u16CCR = (uint16_t)(ChannelConfig->PCLK1_Hz / (3UL * ChannelConfig->ClockSpeed_Hz));
        I2C_Channel[ChannelConfig->Channel]->CCR &= ~(1 << CCR_DUTY);
      }
      else
      {
        Local_u16CCR = (uint16_t)(ChannelConfig->PCLK1_Hz / (25UL * ChannelConfig->ClockSpeed_Hz));
        I2C_Channel[ChannelConfig->Channel]->CCR |= (1 << CCR_DUTY);
      }

      if (Local_u16CCR == 0U)
      {
        Local_u16CCR = 1U; /* minimum allowed CCR value in Fast mode */
      }
      Local_u16TRISE = (uint16_t)((((uint32_t)Local_u8FreqMHz * 300UL) / 1000UL) + 1UL);

      I2C_Channel[ChannelConfig->Channel]->CCR |= (1 << CCR_FS);
    }

    I2C_Channel[ChannelConfig->Channel]->CCR &= ~CCR_CCR_MASK;
    I2C_Channel[ChannelConfig->Channel]->CCR |= (Local_u16CCR & CCR_CCR_MASK);

    I2C_Channel[ChannelConfig->Channel]->TRISE = (Local_u16TRISE & TRISE_MASK);

    /* Configure own address (used if the MCU is addressed as a slave) */
    I2C_Channel[ChannelConfig->Channel]->OAR1 = (uint32_t)(1 << OAR1_MUST_BE_1); /* must always be kept at 1 */
    if (ChannelConfig->AddressMode == I2C_ADDR_MODE_7BIT)
    {
      I2C_Channel[ChannelConfig->Channel]->OAR1 &= ~(1 << OAR1_ADDMODE);
      I2C_Channel[ChannelConfig->Channel]->OAR1 |= (uint32_t)((ChannelConfig->OwnAddress & 0x7FU) << OAR1_ADD1_7);
    }
    else
    {
      I2C_Channel[ChannelConfig->Channel]->OAR1 |= (1 << OAR1_ADDMODE);
      I2C_Channel[ChannelConfig->Channel]->OAR1 |= (uint32_t)(ChannelConfig->OwnAddress & 0x3FFU);
    }

    /* Configure I2C/SMBus mode and SMBus device/host type */
    I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_SMBUS);
    I2C_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->Mode << CR1_SMBUS);
    if (ChannelConfig->Mode == I2C_MODE_SMBUS)
    {
      I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_SMBTYPE);
      I2C_Channel[ChannelConfig->Channel]->CR1 |= (ChannelConfig->SMBusType << CR1_SMBTYPE);
    }

    /* Enable the peripheral */
    I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_PE);

    /* Configure Acknowledge control (only effective while PE = 1) */
    if (ChannelConfig->ACK == I2C_ACK_ENABLE)
    {
      I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_ACK);
    }
    else
    {
      I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_ACK);
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief Master transmit: START -> address (write) -> data bytes -> STOP.
 * @see I2C_interface.h for full documentation.
 */
ErrorState_t I2C_enumMasterTransmit(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress, uint8_t *TxBuffer, uint16_t Size)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint16_t Local_u16Counter;

  if ((ChannelConfig == NULL) || (TxBuffer == NULL))
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else if (ChannelConfig->Channel >= I2C_CHANNEL_COUNT)
  {
    Local_u8ErrorState = NOK;
  }
  else
  {
    Local_u8ErrorState = I2C_enumStart(ChannelConfig);

    if (Local_u8ErrorState == OK)
    {
      Local_u8ErrorState = I2C_enumSendAddress(ChannelConfig, SlaveAddress, I2C_DIRECTION_TRANSMITTER);
    }

    if (Local_u8ErrorState == OK)
    {
      for (Local_u16Counter = 0; Local_u16Counter < Size; Local_u16Counter++)
      {
        Local_u8ErrorState = I2C_enumWriteByte(ChannelConfig, TxBuffer[Local_u16Counter]);
        if (Local_u8ErrorState != OK)
        {
          break;
        }
      }
    }

    /* Always release the bus, even if an error occurred mid-transfer */
    I2C_vStop(ChannelConfig);
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief Master receive: START -> address (read) -> data bytes (NACK +
 *        STOP generated before reading the last byte) -> DR read.
 * @see I2C_interface.h for full documentation.
 */
ErrorState_t I2C_enumMasterReceive(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress, uint8_t *RxBuffer, uint16_t Size)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint16_t Local_u16Counter;
  uint32_t Local_u32TimeoutCounter;

  if ((ChannelConfig == NULL) || (RxBuffer == NULL) || (Size == 0U))
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else if (ChannelConfig->Channel >= I2C_CHANNEL_COUNT)
  {
    Local_u8ErrorState = NOK;
  }
  else
  {
    Local_u8ErrorState = I2C_enumStart(ChannelConfig);

    if (Local_u8ErrorState == OK)
    {
      /* ACK must be enabled before the address phase for multi-byte reads */
      if (Size > 1U)
      {
        I2C_Channel[ChannelConfig->Channel]->CR1 |= (1 << CR1_ACK);
      }
      Local_u8ErrorState = I2C_enumSendAddress(ChannelConfig, SlaveAddress, I2C_DIRECTION_RECEIVER);
    }

    if (Local_u8ErrorState == OK)
    {
      for (Local_u16Counter = 0; Local_u16Counter < Size; Local_u16Counter++)
      {
        if (Local_u16Counter == (uint16_t)(Size - 1U))
        {
          /* Last byte: NACK it and generate STOP before reading DR */
          I2C_Channel[ChannelConfig->Channel]->CR1 &= ~(1 << CR1_ACK);
          I2C_vStop(ChannelConfig);
        }

        Local_u32TimeoutCounter = 0;
        while (((I2C_Channel[ChannelConfig->Channel]->SR1 >> SR1_RXNE) & 1) == 0 &&
               (Local_u32TimeoutCounter != I2C_u32TIMEOUT))
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == I2C_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
          break;
        }

        RxBuffer[Local_u16Counter] = (uint8_t)I2C_Channel[ChannelConfig->Channel]->DR;
      }
    }
    else
    {
      I2C_vStop(ChannelConfig);
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief Write-then-read transaction using a repeated START condition.
 * @see I2C_interface.h for full documentation.
 */
ErrorState_t I2C_enumMasterTransmitReceive(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress,
                                            uint8_t *TxBuffer, uint16_t TxSize,
                                            uint8_t *RxBuffer, uint16_t RxSize)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint16_t Local_u16Counter;

  if ((ChannelConfig == NULL) || (TxBuffer == NULL) || (RxBuffer == NULL))
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    Local_u8ErrorState = I2C_enumStart(ChannelConfig);

    if (Local_u8ErrorState == OK)
    {
      Local_u8ErrorState = I2C_enumSendAddress(ChannelConfig, SlaveAddress, I2C_DIRECTION_TRANSMITTER);
    }

    if (Local_u8ErrorState == OK)
    {
      for (Local_u16Counter = 0; Local_u16Counter < TxSize; Local_u16Counter++)
      {
        Local_u8ErrorState = I2C_enumWriteByte(ChannelConfig, TxBuffer[Local_u16Counter]);
        if (Local_u8ErrorState != OK)
        {
          break;
        }
      }
    }

    if (Local_u8ErrorState == OK)
    {
      /* I2C_enumMasterReceive() issues its own (repeated) START and STOP */
      Local_u8ErrorState = I2C_enumMasterReceive(ChannelConfig, SlaveAddress, RxBuffer, RxSize);
    }
    else
    {
      I2C_vStop(ChannelConfig);
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief Probes a slave address (START + address + STOP) to check
 *        whether it acknowledges, without exchanging any data.
 * @see I2C_interface.h for full documentation.
 */
ErrorState_t I2C_enumIsDeviceReady(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress)
{
  ErrorState_t Local_u8ErrorState = OK;

  if (ChannelConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    Local_u8ErrorState = I2C_enumStart(ChannelConfig);

    if (Local_u8ErrorState == OK)
    {
      Local_u8ErrorState = I2C_enumSendAddress(ChannelConfig, SlaveAddress, I2C_DIRECTION_TRANSMITTER);
    }

    I2C_vStop(ChannelConfig);
  }
  return Local_u8ErrorState;
}
