/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    I2C_interface.h    >>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Alaa Hassan                                     **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : STM32F411CEU6 (BlackPill)                       **
 **                  SWC    : I2C                                             **
 **                                                                           **
 **===========================================================================**
 */
#ifndef _I2C_INTERFACE_H_
#define _I2C_INTERFACE_H_

#include "stdint.h"

/************************** Channel Definitions **************************/
/********************************
 * @I2C_Channel_t enum:
 * @brief: I2C peripheral channel selection
 * @note : STM32F411CEU6 has 3 I2C peripherals (I2C1, I2C2, I2C3)
 */
typedef enum
{
  I2C_CHANNEL1,
  I2C_CHANNEL2,
  I2C_CHANNEL3,
} I2C_Channel_t;

/************************** Peripheral Mode Definitions **************************/
typedef enum
{
  I2C_MODE_I2C,
  I2C_MODE_SMBUS,
} I2C_Mode_t;

typedef enum
{
  I2C_SMBUS_DEVICE,
  I2C_SMBUS_HOST,
} I2C_SMBTYPE_t;

/************************** Acknowledge Control **************************/
typedef enum
{
  I2C_ACK_DISABLE,
  I2C_ACK_ENABLE,
} I2C_ACK_t;

/************************** Addressing Mode **************************/
typedef enum
{
  I2C_ADDR_MODE_7BIT,
  I2C_ADDR_MODE_10BIT,
} I2C_AddMode_t;

/************************** Speed / Duty Cycle **************************/
typedef enum
{
  I2C_SPEED_STANDARD, /* Standard mode, up to 100 KHz  */
  I2C_SPEED_FAST,     /* Fast mode, up to 400 KHz      */
} I2C_SPEED_t;

typedef enum
{
  I2C_DUTY_2,    /* Fm mode t_low/t_high = 2    */
  I2C_DUTY_16_9, /* Fm mode t_low/t_high = 16/9 */
} I2C_FM_DUTY_t;

/************************** Transfer Direction **************************/
typedef enum
{
  I2C_DIRECTION_TRANSMITTER,
  I2C_DIRECTION_RECEIVER,
} I2C_Direction_t;

/************************** Configuration Structure **************************/
/********************************
 * @I2C_Config_t struct:
 * @brief: I2C peripheral configuration structure
 * @param: Channel        - I2C peripheral to configure (I2C_CHANNEL1/2/3)
 * @param: Mode           - I2C or SMBus mode
 * @param: SMBusType      - SMBus device/host type (used only if Mode = SMBUS)
 * @param: ACK            - Enable/disable acknowledge after byte reception
 * @param: AddressMode    - 7-bit or 10-bit own addressing mode
 * @param: OwnAddress     - Own address used when the MCU acts as a slave
 * @param: Speed          - Standard (100 KHz) or Fast (400 KHz) mode
 * @param: FM_DutyCycle   - Duty cycle used only in Fast mode
 * @param: ClockSpeed_Hz  - Target SCL clock speed in Hz (e.g. 100000, 400000)
 * @param: PCLK1_Hz       - APB1 peripheral clock frequency in Hz (from RCC)
 */
typedef struct
{
  I2C_Channel_t   Channel;
  I2C_Mode_t      Mode;
  I2C_SMBTYPE_t   SMBusType;
  I2C_ACK_t       ACK;
  I2C_AddMode_t   AddressMode;
  uint16_t        OwnAddress;
  I2C_SPEED_t     Speed;
  I2C_FM_DUTY_t   FM_DutyCycle;
  uint32_t        ClockSpeed_Hz;
  uint32_t        PCLK1_Hz;
} I2C_Config_t;

/*==================================================================================================*/
/**
 * @fn I2C_enumInit
 * @brief Initialize the I2C peripheral with the provided configuration
 *        (own address, ACK, speed/CCR/TRISE calculation) and enable it.
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @return ErrorState_t OK if initialization successful, error code otherwise
 *
 * @warning NULL pointer check is performed on the input parameter.
 */
ErrorState_t I2C_enumInit(I2C_Config_t *ChannelConfig);

/*==================================================================================================*/
/**
 * @fn I2C_enumMasterTransmit
 * @brief Master transmit: generates START, sends the slave address in
 *        write mode, transmits the whole buffer, then generates STOP.
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @param SlaveAddress  7-bit slave address (right aligned, without R/W bit)
 * @param TxBuffer      Pointer to the data buffer to transmit
 * @param Size          Number of bytes to transmit
 * @return ErrorState_t OK if successful, error code otherwise
 *
 * @note This is a blocking function with timeout protection
 */
ErrorState_t I2C_enumMasterTransmit(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress, uint8_t *TxBuffer, uint16_t Size);

/*==================================================================================================*/
/**
 * @fn I2C_enumMasterReceive
 * @brief Master receive: generates START, sends the slave address in
 *        read mode, receives the whole buffer (NACK + STOP on the last
 *        byte), then generates STOP.
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @param SlaveAddress  7-bit slave address (right aligned, without R/W bit)
 * @param RxBuffer      Pointer to the buffer that will hold received data
 * @param Size          Number of bytes to receive
 * @return ErrorState_t OK if successful, error code otherwise
 *
 * @note This is a blocking function with timeout protection
 */
ErrorState_t I2C_enumMasterReceive(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress, uint8_t *RxBuffer, uint16_t Size);

/*==================================================================================================*/
/**
 * @fn I2C_enumMasterTransmitReceive
 * @brief Combined write-then-read transaction with a repeated START
 *        (typical pattern used to read registers from I2C sensors/EEPROMs:
 *        write register address, repeated START, read N bytes).
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @param SlaveAddress  7-bit slave address (right aligned, without R/W bit)
 * @param TxBuffer      Pointer to the data to transmit before the repeated START
 * @param TxSize        Number of bytes to transmit
 * @param RxBuffer      Pointer to the buffer that will hold received data
 * @param RxSize        Number of bytes to receive
 * @return ErrorState_t OK if successful, error code otherwise
 */
ErrorState_t I2C_enumMasterTransmitReceive(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress,
                                            uint8_t *TxBuffer, uint16_t TxSize,
                                            uint8_t *RxBuffer, uint16_t RxSize);

/*==================================================================================================*/
/**
 * @fn I2C_enumIsDeviceReady
 * @brief Polls a slave address (START + address + STOP) to check whether
 *        the device acknowledges, useful for bus scanning / ready checks.
 *
 * @param ChannelConfig Pointer to the I2C configuration structure
 * @param SlaveAddress  7-bit slave address to probe
 * @return ErrorState_t OK if the device acknowledged, NOK otherwise
 */
ErrorState_t I2C_enumIsDeviceReady(I2C_Config_t *ChannelConfig, uint8_t SlaveAddress);

#endif /* _I2C_INTERFACE_H_ */
