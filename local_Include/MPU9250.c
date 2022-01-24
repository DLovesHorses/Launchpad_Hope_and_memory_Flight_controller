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

    I2C0_Write(0x68, GYRO_CONFIG, 0x18);
}

// For Debug only

void PCF8574A_Init(void)
{
    uint8_t uiData;

    // Configure LEDs and switches
    uiData = 0xE7;  // default

    uiData = 0x47;//0xFF & 0x07;
    PCF8574A_Write( PCF8574A_SA, uiData);

    return;
}

void PCF8574A_Write(uint8_t slaveAddr, uint8_t dataToWr)
{

    // Write slave address and R/W = 0
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
    // true -> read from slave

    // Write data to transmit register
    //I2CMasterDataPut(I2C0_BASE, dataToWr);
    HWREG( I2C0_BASE + I2C_O_MDR ) = dataToWr;

    // Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

    // Wait until the controller is no longer busy
    // wait until bus is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    UARTprintf("PCF written");

    return;
}

/*
void PCF8574A_Read(uint8_t uiSA, uint8_t *puiData)
{
    uint8_t uiMCS;

    // Write slave address and R/W = 1
    HWREG( I2C0_BASE + I2C_O_MSA ) = (uiSA << 1) | 0x01;

    // Initiate I2C transaction
    uiMCS = I2C_MCS_RUN | I2C_MCS_START | I2C_MCS_STOP;
    HWREG( I2C0_BASE + I2C_O_MCS ) = uiMCS;

    // Wait until the controller is no longer busy
    I2C_WaitForControllerReady();

    // Read data from receive register
    *puiData = HWREG(I2C0_BASE + I2C_O_MDR);

    return;
}

*/
