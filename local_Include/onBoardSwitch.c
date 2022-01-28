/*
 * File         :   onBoardSwitch.c
 *
 * Description  :   Houses On-Board switches Initialization and driver.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-24
 */

// Includes
#include "onBoardSwitch.h"

// global variables and externs

// Function definitions.

void SWITCH_Init(void)
{

    //
    // Enable the GPIO port to which the pushbuttons are connected.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    //
    // Set each of the button GPIO pins as an input with a pull-up.
    //
    GPIODirModeSet(GPIO_PORTF_BASE,
    GPIO_PIN_0 | GPIO_PIN_4,
                   GPIO_DIR_MODE_IN);

    GPIOPadConfigSet(GPIO_PORTF_BASE,
    GPIO_PIN_0 | GPIO_PIN_4,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);

    // GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4); // this cannot be used since we need a pull-up register

}

bool SWITCH_SW1_Pressed(void)
{
    bool result;
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0)
    {
        result = true;
    }
    else
    {
        result = false;
    }

    return result;

}

bool SWITCH_SW2_Pressed(void)
{
    bool result;
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 0)
    {
        result = true;
    }
    else
    {
        result = false;
    }

    return result;

}
