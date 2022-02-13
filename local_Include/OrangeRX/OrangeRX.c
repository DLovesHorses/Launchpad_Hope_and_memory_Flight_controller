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
#include "local_Include/SysFlag.h"
#include "local_Include/BUZZER/buzzer.h"

// global variables and externs

Orange_RX_Channel_Frequency_Data rx_data;
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
    SysFlag_Set(Orange_RX_INT);

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
                 TM4C_CLK_RATE);
    TimerMatchSet(WTIMER5_BASE,
    TIMER_A,
                  0x00);

    // Configure Timer events (detect both edges)
    TimerControlEvent(WTIMER5_BASE,
    TIMER_A,
                      TIMER_EVENT_POS_EDGE);

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
 *
 *
 * This function shows individual channel data from receiver.
 */

void OrangeRX_showData(void)
{

    double dataToShow = 0;
    char cBuffer[256];
    cBuffer[0] = '\0';

    sprintf(cBuffer,
            "(Ch. 1 : %5f, Ch.2 : %5f, Ch.3 : %5f, Ch.4 : %5f, Ch.5 : %5f, Ch.6 : %5f, Frame_Gap : %5f",
            rx_data.ch1_freq, rx_data.ch2_freq, rx_data.ch3_freq,
            rx_data.ch4_freq, rx_data.ch5_freq, rx_data.ch6_freq,
            rx_data.frame_gap_freq);

    UARTprintf("%s\n\n", cBuffer);

    // Channel 1
    //dataToShow = rx_data.ch1_freq;
    //sprintf(cBuffer,"Channel 1:")

}
