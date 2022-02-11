/*
 * File         :   OrangeRX.c
 *
 * Description  :   This file contains all required initialization, configuration,
 *                  access, and interrupt handling code
 *                  for OrangeRX cPPM signal.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-11
 */

// Includes
#include "OrangeRX.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"
#include "local_Include/BUZZER/buzzer.h"

// global variables and externs

// Function definitions.

/*
 *
 *
 *
 *
 *
 */

void OrangeRX_IntHandler(void)
{

    TimerIntClear(WTIMER5_BASE, TIMER_CAPA_EVENT);
    static bool ledState = 0;
    ledState = ledState ^ 0x01;

    static uint32_t firstMeasure = 0;
    static uint32_t secondMeasure = 0;
    uint32_t        difference = 0;



    firstMeasure = secondMeasure;
    secondMeasure = TimerValueGet(WTIMER5_BASE, TIMER_A);

#ifdef DEBUG
    //BUZZ_BUZZER(BUZZ_DUR_LONG, BUZZ_REP_TIME_4,
    // BUZZ_PAUSE_TIME_100);

    difference = (uint32_t) (secondMeasure - firstMeasure);

    char cBuffer[100];
    sprintf(cBuffer, "Time between edges: %d. \n\n", difference);
    UARTprintf("%s", cBuffer);
    LED_LED1(ledState);
#endif

    TimerIntEnable(WTIMER5_BASE, TIMER_CAPA_EVENT);

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
 * Configure PD6 as CCP (Capture Compare PWM pin)
 * to capture Signal edges on pin PD6.
 *
 */
void OrangeRX_Init(void)
{

    // Enable Wide Timer 5 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);

    // Enable and configure GPIO pin PD6 as CCP pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinConfigure(GPIO_PD6_WT5CCP0);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6,
    GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // Configure Timer peripheral in capture mode.
    TimerConfigure(WTIMER5_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME));
    TimerLoadSet(WTIMER5_BASE,
    TIMER_A,
                 0xFFFF);
    TimerMatchSet(WTIMER5_BASE,
    TIMER_A,
                  0x00);

    // Configure Timer events (detect both edges)
    TimerControlEvent(WTIMER5_BASE,
    TIMER_A,
                      TIMER_EVENT_BOTH_EDGES);

    // Configure interrupts that fires when the
    // level of external signal changes (when edge occours).

    TimerIntRegister(WTIMER5_BASE,
    TIMER_A,
                     OrangeRX_IntHandler);

    // IntEnable(INT_WTIMER5A);
    TimerIntEnable(WTIMER5_BASE, TIMER_CAPA_EVENT);

    // Finally, enable Timer
    TimerEnable(WTIMER5_BASE, TIMER_A);

    return;
}
