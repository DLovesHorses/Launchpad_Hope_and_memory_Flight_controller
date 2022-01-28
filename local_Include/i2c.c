/*
 * File         :   i2c.c
 *
 * Description  :   Houses functions for I2C communication support.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-27
 */

// Includes

#include "i2c.h"
#include "uart.h"


// global variables and externs



// Function definitions.

void I2C0_IntHandler(void)
{

}

void I2C0_Init(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE,
    GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE,
    GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), // bFast true: 400 kHz, bFast False: 100 kHz
                        true);

#ifdef DEBUG
    // DEBUG: <Component> | <File>   | <Routine> |   <Operation> | <Status>  |   <Data>
    UARTprintf("I2C0 Initialized");           // defines data
    UARTprintf("\n\n");
#endif

}

void I2C_ReadByte(uint8_t slaveAddr, uint32_t regAddr, uint8_t *puiData) // working
{

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
// true -> read from slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

// Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    true);  // false -> write to slave;
            // true -> read from slave

    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

// Read data from receive register
    uint32_t receivedData;
    receivedData = I2CMasterDataGet(I2C0_BASE);

    *puiData = receivedData;
    /*
     #ifdef DEBUG
     // DEBUG: <Component> | <File>   | <Routine> |   <Operation> | <Status>  |   <Data>
     //UARTdebug("I2C0", "i2c.c", "I2C0_ReadByte", "Reading slave register", "SUCCESS");
     UARTprintf("Received data from: Slave: %x, Reg: %x, Data: %x", slaveAddr, regAddr, *puiData);           // defines data
     UARTprintf("\n");
     #endif
     */
    return;
}

// For Debug only

void I2C_AddressBruteForcer(uint8_t knownReg, uint8_t knownValue)
{
    uint8_t testAddr = 0x00;
    bool found = 0;
    uint8_t dataReceived[1];

    UARTprintf("\n\nRunning Slave Address Brute forcer on I2C0... ");
    UARTprintf("\n\nTarget Register: %X \n", knownReg);
    UARTprintf("\n\nKnown Value: %X \n", knownValue);

    for(testAddr = 0x00; testAddr < 0xFF; testAddr++){
        I2C_ReadByte(testAddr, knownReg, dataReceived);

        if(dataReceived[0] == knownValue){
            found = 1;
            UARTprintf("\nSlave address found: %x", testAddr);
        }
    }

    if (found == 0){
        UARTprintf("\nSlave not on the bus.", testAddr);
    }

    return;
}

void PCF8574A_Init(void)
{
    uint8_t uiData;

// Configure LEDs and switches
    uiData = 0xE7;  // default

    uiData = 0x47;  //0xFF & 0x07;
    PCF8574A_Write( PCF8574A_SA, uiData);

#ifdef DEBUG
    // DEBUG: <Component> | <File>   | <Routine> |   <Operation> | <Status>  |   <Data>
    //UARTdebug("I2C0", "i2c.c", "I2C0_ReadByte", "Reading slave register", "SUCCESS");
    UARTprintf("PCF8574 Initialized");           // defines data
    UARTprintf("\n");
#endif
    return;
}

void PCF8574A_Write(uint8_t slaveAddr, uint8_t dataToWr)
{

// Write slave address and R/W = 0
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
// true -> read from slave

// check if i2c Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {
// wait if bus is busy
    }

// Write data to transmit register
    I2CMasterDataPut(I2C0_BASE, dataToWr);
//HWREG( I2C0_BASE + I2C_O_MDR ) = dataToWr;

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

// Wait until the controller is no longer busy
// wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

#ifdef DEBUG
    // DEBUG: <Component> | <File>   | <Routine> |   <Operation> | <Status>  |   <Data>
    //UARTdebug("I2C0", "i2c.c", "PCF8574A_Write", "Writing PCF8574", "SUCCESS");
    UARTprintf("PCF written");           // defines data
    UARTprintf("\n");
#endif

    return;
}

void PCF8574A_Read(uint8_t slaveAddr, uint32_t *puiData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    true);         // false -> write to slave;
// true -> read from slave

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_RECEIVE);

// Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

// Read data from receive register
    uint32_t receivedData;
    receivedData = I2CMasterDataGet(I2C0_BASE);

    *puiData = receivedData;

#ifdef DEBUG
    // DEBUG: <Component> | <File>   | <Routine> |   <Operation> | <Status>  |   <Data>
    //UARTdebug("I2C0", "i2c.c", "PCF8574A_Read", "Reading PCF8574", "SUCCESS");
    UARTprintf("Data received from: SA: %x (PCF), data:   %x", slaveAddr,
               *puiData);           // defines data
    UARTprintf("\n");
#endif
    return;
}
