/*
 * File         :   buzzer.c
 *
 * Description  :   <Short Discription of the functionality provided.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-06
 */

// Includes
#include "buzzer.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"

// global variables and externs

bool BUZZER_state = NOT_INITIALIZED;

uint16_t buzDuration;
uint16_t count;
uint16_t pauseTime;


/*
 *
 *
 *
 *
 *
 * BUZZER -> PC_7
 *
 *
 */
void init_Buzzer(void)
{

    // enable GPIO port C system peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //
    // Check if the peripheral access is enabled.
    //
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA,
    GPIO_PIN_TYPE_STD);
    // configure PC7 as output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);


    BUZZER_state = INITIALIZED;
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
 */
void BUZZER_showState(void){

    if( BUZZER_state  == INITIALIZED){
        UARTprintf("BUZZER  initialized. \n");
    }
    else{
        UARTprintf("BUZZER not initialized. \n");
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
 *
 *
 *
 */
void BUZZ_BUZZ(bool buzzTone)
{

    if (buzzTone == ON)
    {
        // turn on Buzzer.
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, OFF);
    }

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

void BUZZ_BUZZER(uint16_t duration, uint8_t repTime, uint16_t pauseIntervalTime)
{

    buzDuration     = duration;           // duration of Buzz
    count           = repTime;            // number of time to repeat
    pauseTime       = pauseIntervalTime;  // number of ms to wait after each BUZZ.

    BUZZ_SM(BUZZ_SM_CALLED_FROM_BUZZ);

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
 */
void BUZZ_SM(bool callerID)
{

    static uint8_t curState = BUZZ_OFF;
    static uint8_t nextState = BUZZ_OFF;
    static uint8_t stepCount = 0;

    static uint16_t on_time;
    static uint16_t pause_time;

    if (callerID == BUZZ_SM_CALLED_FROM_MAIN)
    {
        // decrement pauseTime by 1

    }

    if (callerID == BUZZ_SM_CALLED_FROM_BUZZ)
    {

        curState = BUZZ_ON;
        stepCount = count - 1;

    }

    switch (nextState)
    {

    case BUZZ_OFF:
    {
        // check if the SM needs to transition to BUZ_ON
        if (curState == BUZZ_ON)
        {
            nextState = BUZZ_ON;
            on_time = buzDuration;
            BUZZ_BUZZ(ON);
            curState = BUZZ_OFF;
        }

        break;
    }

    case BUZZ_ON:
    {
        if (on_time > 0)
        {
            on_time--;
        }

        else
        {
            // goto BUZZ_PAUSE state
            nextState = BUZZ_PAUSE;
            pause_time = pauseTime;
            BUZZ_BUZZ(OFF);
        }

        break;
    }

    case BUZZ_PAUSE:
    {
        if (pause_time > 0)
        {
            pause_time--;
            // do nothing (remain in BUZZ_PAUSE state)
        }
        else
        {
            // current pass complete, decrement count
            if (stepCount != 0)
            {
                stepCount--;
            }
            else{
                stepCount = 0;
            }

            // if more pass remaining, goto BUZZ_ON and repeat
            if (stepCount >= 1)
            {
                nextState = BUZZ_ON;
                on_time = buzDuration;
                BUZZ_BUZZ(ON);
            }

            // else, goto BUZZ_OFF state
            else
            {
                nextState = BUZZ_OFF;
            }

        }
        break;
    }

    }

}
