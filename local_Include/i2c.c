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


