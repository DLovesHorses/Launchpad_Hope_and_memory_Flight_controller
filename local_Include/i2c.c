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

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), // bFast true: 400 kHz, bFast False: 100 kHz
                        true);
    UARTprintf("I2C0 Initialized.\n");
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

    return;
}

