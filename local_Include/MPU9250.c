#include "MPU9250.h"
#include "i2c.h"

void MPU9250_Init(void)
{
    /*
     * Slave address    : 110100X
     *      X           : Level of SA0 pin on the breakout
     *                  : 0 -> current config
     *
     * Current SA (Hex) : 68
     */

    MPU9250_Write(GYRO_CONFIG, 0x18);
}

void MPU9250_Write(uint8_t regAddr, uint8_t dataToWr)
{

    uint32_t errorStatus;
    UARTprintf("Writing MPU9250: Register %X with Data %X\n", regAddr,
               dataToWr);



    // append write command at the end of slave address
    // (handled in I2CMasterSlaveAddrSet())
    //slaveAddr = (slaveAddr << 1) | 0x00 ;       // 0x00 -> write;
    // 0x01 -> read.

    // set I2C0 slave address

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
    false);         // false -> write to slave;
                    // true -> read from slave

    // put address of register to be written into slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

    // send register address

    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

    // wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    errorStatus = I2CMasterErr(I2C0_BASE);

    if (errorStatus == I2C_MASTER_ERR_NONE)
    {
        UARTprintf("MPU9250 on the bus. \n", regAddr, dataToWr);

        // put data to be transmitted to slave

        I2CMasterDataPut(I2C0_BASE, dataToWr);

        // send data

        I2CMasterControl(I2C0_BASE,
        I2C_MASTER_CMD_SINGLE_SEND);

        while (I2CMasterBusBusy(I2C0_BASE))
        {

        }

        errorStatus = I2CMasterErr(I2C0_BASE);

        if (errorStatus == I2C_MASTER_ERR_DATA_ACK)
        {
            UARTprintf("MPU9250 on the bus, but data error.\n");
        }
        else
        {
            UARTprintf("MPU9250 written: Register %X with Data: %X\n", regAddr,
                       dataToWr);
        }
    }

    else if (errorStatus == I2C_MASTER_ERR_ADDR_ACK)
    {
        UARTprintf("MPU9250 not on the bus.\n");
    }

    else{
        UARTprintf("MPU9250 error: UNKNOWN\n");
    }

}

// For Debug only

void PCF8574A_Init(void)
{
    uint8_t uiData;

// Configure LEDs and switches
    uiData = 0xE7;  // default

    uiData = 0x47;  //0xFF & 0x07;
    PCF8574A_Write( PCF8574A_SA, uiData);

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

    UARTprintf("PCF written\n");

    return;
}

void PCF8574A_Read(uint8_t slaveAddr, uint32_t *puiData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    true);         // false -> write to slave;
// true -> read from slave

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

// Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

// Read data from receive register
    uint32_t receivedData;
    receivedData = I2CMasterDataGet(I2C0_BASE);

    *puiData = receivedData;

    return;
}

