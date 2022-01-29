/*
 * File         :   i2c.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for i2c.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-27
 */

// Includes
#include "local_include/global.h"

// Defines

#define I2C_RUN     0x00000001
#define I2C_START   0x00000002
#define I2C_STOP    0x00000004
#define I2C_ACK     0x00000008

// Macros

// Function prototypes

void I2C0_IntHandler(void);
void I2C0_Init(void);

void I2C_ReadByte(uint8_t slaveAddr, uint32_t regAddr, uint8_t *puiData);
bool I2C_AddressBruteForcer(uint8_t knownReg, uint8_t knownValue);

// For Debug only

// Slave Address
#define PCF8574A_SA 0x38

// P7 = 1: LED4
// P6 = 1: LED5
// P5 = 1: LED6
// P4 = 0: Not Used (Reserved 1)
// P3 = 0: Not Used (Reserved 2)
// P2 = 1: PB6
// P1 = 1: PB5
// P0 = 1: PB4

#define PCF8574A_LED4 0x80
#define PCF8574A_LED5 0x40
#define PCF8574A_LED6 0x20
#define PCF8574A_RES2 0x10
#define PCF8574A_RES1 0x08
#define PCF8574A_PB6  0x04
#define PCF8574A_PB5  0x02
#define PCF8574A_PB4  0x01

#define PCF8574A_OUTPUTS ( PCF8574A_LED4 | PCF8574A_LED5 | PCF8574A_LED6 | PCF8574A_RES1 | PCF8574A_RES2 )
#define PCF8574A_INPUTS ( PCF8574A_PB6 | PCF8574A_PB5 | PCF8574A_PB4 )

#define PCF8574A_CONFIG ( PCF8574A_OUTPUTS | PCF8574A_INPUTS )

void PCF8574A_Init(void);
void PCF8574A_Write(uint8_t uiSA, uint8_t uiData);
void PCF8574A_Read(uint8_t uiSA, uint32_t *puiData);

void I2C_readReg(uint8_t slaveAddr, uint32_t regAddr, uint8_t *pBuffer,
                 uint16_t len);

void I2C_writeSingleReg(uint8_t slaveAddr, uint8_t regAddr, uint8_t byteToWrite);
