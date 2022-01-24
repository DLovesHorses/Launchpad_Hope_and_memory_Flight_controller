#include "i2c.h"

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

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(),
    false);
    UARTprintf("I2C0 Initialized.\n");
}

void I2C0_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t dataToWr)
{
    // append write command at the end of slave address
    // (handled in I2CMasterSlaveAddrSet())
    //slaveAddr = (slaveAddr << 1) | 0x00 ;       // 0x00 -> write;
    // 0x01 -> read.

    // set I2C0 slave address

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
                    // true -> read from slave

    // check if i2c bus is busy
    while (I2CMasterBusBusy(I2C0_BASE))
    {
        // wait if bus is busy
    }

    // put address of register to be written into slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

    // send register address

    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

    // wait until bus is not busy
    while (I2CMasterBusBusy(I2C0_BASE))
    {

    }

    if ((HWREG( I2C0_BASE + I2C_O_MCS ) & I2C_MCS_ADRACK) == 0)
    {
        UARTprintf("\nChip Responding\n");
    }
    else{
        UARTprintf("\nChip not on the bus.\n");
    }

    // put data to be transmitted to slave

    I2CMasterDataPut(I2C0_BASE, dataToWr);

    // send data

    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

    while (I2CMasterBusBusy(I2C0_BASE))
    {

    }

    if ((HWREG( I2C0_BASE + I2C_O_MCS ) & I2C_MCS_DATACK) == 0)
    {
        UARTprintf("\nChip Accepted data.\n");
    }

}
