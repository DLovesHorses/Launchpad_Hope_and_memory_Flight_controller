/*
 * File         :   <fileName>
 *
 * Description  :   <Short Discription of the functionality provided.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */

// Includes
#include "SONAR.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"
#include "local_Include/SysFlag.h"

// global variables and externs

// Function definitions.
/*
 *
 *
 *
 *
 */
void SONAR_TrigIntHandler(void)
{

    TimerIntClear(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);
    SysFlag_Set(SONAR_TRIG_INT);

    UARTprintf("WT3CCP1 triggered.\n");
    SONAR_Trigger(LOW, TRIG_LOW_TIME);
    //TimerIntEnable(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

    return;
}
/*
 *
 *
 *
 *
 *
 */

void SONAR_EchoIntHandler(void){

    TimerIntClear(WTIMER3_BASE, TIMER_CAPA_EVENT);

    TimerIntEnable(WTIMER3_BASE, TIMER_CAPA_EVENT);

    static uint32_t count = 1;
    UARTprintf("Echo called : \t %d \n", count);
    count++;
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
void SONAR_Init(void)
{
    // GPIO PD3 -> Output   (Trigger pin);
    // GPIO PD2 -> Input    (Echo Pin);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER3))
    {
    }


    // for PD3 (trig)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);

    // for PD2 (echo)
    GPIOPinConfigure(GPIO_PD2_WT3CCP0);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_2);
    //GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    TimerConfigure(WTIMER3_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_ONE_SHOT));

    TimerLoadSet(WTIMER3_BASE, TIMER_A, 79999759);
    TimerMatchSet(WTIMER3_BASE, TIMER_A, 0x00);
    TimerControlEvent(WTIMER3_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    TimerIntRegister(WTIMER3_BASE, TIMER_A, SONAR_EchoIntHandler);
    TimerIntEnable(WTIMER3_BASE, TIMER_CAPA_EVENT);
    TimerEnable(WTIMER3_BASE, TIMER_A);





    // create a timer for Trigger (10 uS)
    // PD3 -> WT3CCP1


    // Configure Timer peripheral in capture mode.
    //TimerConfigure(WTIMER3_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT));
    TimerLoadSet(WTIMER3_BASE,
    TIMER_B,
                 SysCtlClockGet() / TRIG_TIMER_DIV_1_S);

    TimerIntRegister(WTIMER3_BASE,
    TIMER_B,
                     SONAR_TrigIntHandler);

    TimerIntEnable(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);

    // create the input-edge Capture timer to get the pulse duration

}

/*
 *
 *
 *
 *
 *
 *
 */
void SONAR_Trigger(bool trigState, float trigTime)
{
    if (trigState == LOW)
    {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
        return;
    }
    //else
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);

    TimerLoadSet(WTIMER3_BASE,
    TIMER_B,
                 SysCtlClockGet() / trigTime);
    TimerEnable(WTIMER3_BASE, TIMER_B);

    return;
}
