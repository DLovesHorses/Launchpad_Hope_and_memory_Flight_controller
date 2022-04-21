/*
 * File         :   PPM.c
 *
 * Description  :   This file contains the functions responsible of recreating the PPM signals. This signals are actually the input of ESCs.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */

// Includes
#include "PPM.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"

// global variables and externs

uint8_t curSelectedESC = ESC_1; // Initially ESC 1 is selected.


// Function definitions.
void PPM_IntHandler(void)
{

    TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Make curSelectedESC GPIO LOW

    switch (curSelectedESC)
    {

    case ESC_1:
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);  // PE2 (LOW)
        break;
    }
    case ESC_2:
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x00);  // PE3 (LOW)
        break;
    }
    case ESC_3:
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x00);  // PA2 (LOW)
        break;
    }
    case ESC_4:
    {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);  // PA3 (LOW)
        break;
    }

    default:
    {

        break;
    }

    }

    // increment current selected ESC automatically.
    curSelectedESC++;

    if(curSelectedESC > ESC_4){
        curSelectedESC = ESC_1;
    }

    //UARTprintf("PPM int handler called. \n");
    return;
}
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void PPM_Init(void)
{

    // configure Motor PPM input
    PPM_SignalOutPin_Init();

    // configure PC4 (WT0CCP0) as one-shot periodic timer.

    // Enable Wide Timer 0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    // Enable and configure GPIO pin PC4 as CCP pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC4_WT0CCP0);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // Configure Timer peripheral in One-shot mode.
    TimerConfigure(
            WTIMER0_BASE,
            (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_A_ACT_SETTO));


    // Configure interrupts that fires when the
    // level of external signal changes (when edge occurs).

    TimerIntRegister(WTIMER0_BASE, TIMER_A, PPM_IntHandler);

    // IntEnable(INT_WTIMER5A);
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Finally, enable Timer
    //TimerLoadSet(WTIMER0_BASE, TIMER_A, SysCtlClockGet());
    //TimerEnable(WTIMER0_BASE, TIMER_A);

#ifdef DEBUG
    UARTprintf("Timer for ESC configured. \n");

#endif

    return;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 * PE2  ->  ESC1    (Motor 1)
 * PE3  ->  ESC2    (Motor 2)
 * PA2  ->  ESC3    (Motor 3)
 * PA3  ->  ESC4    (Motor 4)
 *
 */
void PPM_SignalOutPin_Init(void)
{

    // Configure GPIO output pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable GPIO pins
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);     // ESC 1
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);     // ESC 2

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);     // ESC 3
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);     // ESC 4

    return;

}
/*
 *
 *
 *
 *
 *
 *
 * PE2  ->  ESC1    (Motor 1)
 * PE3  ->  ESC2    (Motor 2)
 * PA2  ->  ESC3    (Motor 3)
 * PA3  ->  ESC4    (Motor 4)
 *
 */

void PPM_ESC_Control(uint8_t escID, bool state)
{

    if (state == ON)
    {

        switch (escID)
        {

        case ESC_1:
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PE2
            break;
        }
        case ESC_2:
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);  // PE3
            break;
        }
        case ESC_3:
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PA2
            break;
        }
        case ESC_4:
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);  // PA3
            break;
        }

        default:
        {

            break;
        }

        }

    }

    else
    {
        switch (escID)
        {

        case ESC_1:
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);  // PE2 (LOW)
            break;
        }
        case ESC_2:
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x00);  // PE3 (LOW)
            break;
        }
        case ESC_3:
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x00);  // PA2 (LOW)
            break;
        }
        case ESC_4:
        {
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);  // PA3 (LOW)
            break;
        }

        default:
        {

            break;
        }

        }
    }


    return;

}


/*
 *
 *
 *
 *
 *
 *
 *
 */

void PPM_ESC_HighLevelControl(uint8_t curESC, uint32_t loadValue){

    // set current esc high
    PPM_ESC_Control(curESC, ON);

    // Enable timer for loadValue
    TimerLoadSet(WTIMER0_BASE, TIMER_A, loadValue);
    TimerEnable(WTIMER0_BASE, TIMER_A);








}

