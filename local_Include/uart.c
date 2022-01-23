#include "uart.h"
#include "led.h"

void UART0_IntHandler(void)
{

    uint32_t interruptReason = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, interruptReason);

    if (interruptReason == UART_INT_RX)
    {
        SysFlag_Set(SYSFLAG_UART0_RX);
    }

    else if (interruptReason == UART_INT_TX)
    {
        SysFlag_Set(SYSFLAG_UART0_TX);
    }
    else
    {

    }
    LED_LED3(ON);
    uint32_t count = 0;
    for (count = 0; count < 20000; count++)
    {

    }

    LED_ALL(OFF);
}

void UART0_STDIO_IntHandler(void){

    uint32_t interruptReason = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, interruptReason);
    return;
}


void UART0_STDIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTIntRegister(UART0_BASE, UART0_STDIO_IntHandler);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    UARTFlushRx();
    UARTFlushTx(true);
    //UARTprintf("Hello");
    //UARTEchoSet(false);
}

void UART0_Init(void)
{

    // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

    UARTConfigSetExpClk(
            UART0_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTFIFOEnable(UART0_BASE);
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    HWREG( UART0_BASE + 0x00000018 ) = 0;

    UARTIntRegister(UART0_BASE, UART0_IntHandler);
    UARTIntEnable(UART0_BASE, UART_INT_TX | UART_INT_RX);

    //UARTCharPut(UART0_BASE, 'a');

    /*
     UARTStdioConfig(0, 115200, SysCtlClockGet());
     UARTprintf("Hello world!\n");
     */
}
