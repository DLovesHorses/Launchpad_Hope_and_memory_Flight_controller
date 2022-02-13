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
uint8_t frameGapChNo = 0;
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

    char cBuffer[256];
    cBuffer[0] = '\0';

    UARTprintf(
            "Channel # \t     \t \t \t Physical \t \t \t \t \t \t Logical \n\n");
    uint8_t count = 0;
    for (count = 0; count < 7; count++)
    {

        /*// Only logical channel data
         sprintf(cBuffer,
         "Ch. [%1d] \t -> \t Freq:    %5f , \t period:    %10f \n",
         count, *(rx_data.pFreq_Ch[count]), *(rx_data.pPeriod_Ch[count]));
         */

        /*// only Physical channel data

        sprintf(cBuffer,
         "Ch. [%1d] \t -> \t Freq:    %5f , \t period:    %10f \n",
         count, rx_data.ch_freq[count], rx_data.ch_period[count], );

         */

        // both
        sprintf(cBuffer,
                "Ch. [%1d] \t -> \t Freq:    %05f , \t period:    %05f \t \t Freq:    %05f , \t period:    %05f \n",
                count, rx_data.ch_freq[count], rx_data.ch_period[count],
                *(rx_data.pFreq_Ch[count]), *(rx_data.pPeriod_Ch[count]));

        UARTprintf("%s", cBuffer);
        cBuffer[0] = '\0';
    }

    UARTprintf("\nMax Period Channel = %d", frameGapChNo);
    UARTprintf("\n\n");
    /*
     sprintf(cBuffer, "Ch.1 : \t %5f \n", rx_data.ch1_freq);
     "Ch.1 : \t %5f \nCh.2 : \t %5f \nCh.3 : \t %5f \nCh.4 : \t %5f \nCh.5 : \t %5f \nCh.6 : %5f \nFrame_Gap : %5f",
     rx_data.ch1_freq, rx_data.ch2_freq, rx_data.ch3_freq,
     rx_data.ch4_freq, rx_data.ch5_freq, rx_data.ch6_freq,
     rx_data.frame_gap_freq);

     UARTprintf("%s\n\n", cBuffer);
     */

    // Channel 1
    //dataToShow = rx_data.ch1_freq;
    //sprintf(cBuffer,"Channel 1:")
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
 * This function arranges the received data in logical order.
 * the trick is that the frequency of FRAME_GAP will always be minimum
 * that is, the period is always maximum, as it can be observed in oscilloscope.

 * compare the just received frame and then rearrange the pointers
 * so that the first pointer points to the channel with max period (or min freq),
 * the second pointer points to the value just next (CH_1 logically), and so on.

 *
 *
 *
 *
 */

void OrangeRX_receivedChannelReorganizer(void)
{

    // Part 1: Goal: point to the channel containing max. period
    uint8_t count = 0;
    double maxPeriod = 0; // assume that 0 has the max.
    //double *pFrameGapCh = &(rx_data.ch_period[0]); // initially point to first channel <Frame_GAP>

    // find the max. period among the acquired data
    for (count = 0; count < 7; count++)
    {

        if (rx_data.ch_period[count] > maxPeriod)
        {
            maxPeriod = rx_data.ch_period[count];
            //pFrameGapCh = &(rx_data.ch_period[count]);
            frameGapChNo = count;
        }
    }

    /* now, frameGapChNo contains the physical channel number
     * of the channel holding the data from logical frame gap channel
     * using this, and a little bit of magic, point appropriate
     * pointers to their respective logical channels.
     *
     * That is, pointer assignment will be such that:
     * pPeriod_Ch[0] = FRAME_GAP
     * pPeriod_Ch[1] = CH_1
     * pPeriod_Ch[2] = CH_2
     * pPeriod_Ch[3] = CH_3
     * pPeriod_Ch[4] = CH_4
     * pPeriod_Ch[5] = CH_5
     * pPeriod_Ch[6] = CH_6
     *
     *
     */

    // start the magic
    for (count = 0; count < 7; count++)
    {
        rx_data.pPeriod_Ch[count] = &rx_data.ch_period[(frameGapChNo + count)
                % 7];
        rx_data.pFreq_Ch[count] = &rx_data.ch_freq[(frameGapChNo + count) % 7];
    }
    // that's it!...
    // now the channels are logically organized in desired order.

    // from now on, use only pPeriod_Ch variable whenever you want to access the data.

}
