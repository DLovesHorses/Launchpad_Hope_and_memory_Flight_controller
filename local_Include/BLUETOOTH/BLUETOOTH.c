/*
 * File         :   BLUETOOTH.c
 *
 * Description  :   This file contains all functions for supporting Bluetooth module.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-26
 */

// Includes
#include "BLUETOOTH.h"
#include "local_Include/led.h"
#include "local_include/SysFlag.h"

// global variables and externs

extern float roll_kp_tune;
extern float roll_ki_tune;

extern float pitch_kp_tune;
extern float pitch_ki_tune;

extern float yaw_kp_tune;
extern float yaw_ki_tune;

extern float alt_kp_tune;
extern float alt_ki_tune;

// buffer to store the string to send via bluetooth
char charBuffer[256] = { '0' };

// Function definitions.
/*
 *
 *
 *
 *
 *
 *
 *
 */
void BLUETOOTH_IntHandler(void)
{
    uint32_t intStatus = UARTIntStatus(UART5_BASE, true);

    UARTIntClear(UART5_BASE, UART_INT_RX);

    int32_t charRec = UARTCharGet(UART5_BASE);

    switch (charRec)
    {

    case ROLL_KP_PLUS:
    {
        roll_kp_tune += 0.01;
        sprintf(charBuffer, "Roll Kp : %f \n", roll_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';
        //BLUETOOTHprintf("Roll Kp+ pressed.");
        break;
    }

    case ROLL_KP_MINUS:
    {
        roll_kp_tune -= 0.01;
        sprintf(charBuffer, "Roll Kp : %f \n", roll_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Roll Kp- pressed.");
        break;
    }

    case ROLL_KI_PLUS:
    {
        roll_ki_tune += 0.0001;
        sprintf(charBuffer, "Roll Ki : %f \n", roll_ki_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Roll Ki+ pressed.");
        break;
    }

    case ROLL_KI_MINUS:
    {
        roll_ki_tune -= 0.0001;
        sprintf(charBuffer, "Roll Ki : %f \n", roll_ki_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Roll Ki- pressed.");
        break;
    }

    case PITCH_KP_PLUS:
    {
        pitch_kp_tune += 0.01;
        sprintf(charBuffer, "Pitch Kp : %f \n", pitch_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Pitch Kp+ pressed.");
        break;
    }

    case PITCH_KP_MINUS:
    {
        pitch_kp_tune -= 0.01;
        sprintf(charBuffer, "Pitch Kp : %f \n", pitch_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Pitch Kp- pressed.");
        break;
    }

    case PITCH_KI_PLUS:
    {
        pitch_ki_tune += 0.0001;
        sprintf(charBuffer, "Pitch Ki : %f \n", pitch_ki_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Pitch Ki+ pressed.");
        break;
    }

    case PITCH_KI_MINUS:
    {
        pitch_ki_tune -= 0.0001;
        sprintf(charBuffer, "Pitch Ki : %f \n", pitch_ki_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Pitch Ki- pressed.");
        break;
    }

    case HEIGHT_KP_PLUS:
    {
        alt_kp_tune += 0.01;
        sprintf(charBuffer, "Altitude Kp : %f \n", alt_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Height Kp+ pressed.");
        break;
    }

    case HEIGHT_KP_MINUS:
    {
        alt_kp_tune -= 0.01;
        sprintf(charBuffer, "Altitude Kp : %f \n", alt_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Height Kp- pressed.");
        break;
    }

    case HEIGHT_KI_PLUS:
    {
        alt_ki_tune += 0.0001;
        sprintf(charBuffer, "Altitude Ki : %f \n", alt_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Height Ki+ pressed.");
        break;
    }

    case HEIGHT_KI_MINUS:
    {
        alt_ki_tune -= 0.0001;
        sprintf(charBuffer, "Altitude Ki : %f \n", alt_kp_tune);
        BLUETOOTHprintf(charBuffer);
        charBuffer[0] = '\0';

        //BLUETOOTHprintf("Height Ki- pressed.");
        break;
    }

    default:
    {
        // do nothing
    }

    }

    UARTConfigSetExpClk(
            UART5_BASE, SysCtlClockGet(), BLUETOOTH_BAUD_RATE,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

#ifdef DEBUG
//UARTprintf("Bluetooth interrupt received : 0x%X \n", intStatus);

    UARTprintf("Char Received: %c\n", charRec);
    charBuffer[0] = '\0';
    sprintf(charBuffer, "Roll Kp \t -> \t %f\nRoll Ki \t -> \t %f", roll_kp_tune,
            roll_ki_tune);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';
    UARTprintf("\n");

    sprintf(charBuffer, "Pitch Kp \t -> \t %f\nPitch Ki \t -> \t %f", pitch_kp_tune,
            pitch_ki_tune);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';
    UARTprintf("\n");

    sprintf(charBuffer, "Yaw Kp \t \t -> \t %f\nYaw Ki \t \t -> \t %f", yaw_kp_tune,
            yaw_ki_tune);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';
    UARTprintf("\n");

    sprintf(charBuffer, "Altitude Kp \t -> \t %f\nAltitude Ki \t -> \t %f", alt_kp_tune,
            alt_ki_tune);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';
    UARTprintf("\n\n\n");



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
 *
 *
 *
 *
 */

void BLUETOOTH_Init(void)
{
// Setup UART 05 module (PE4 = RX, PE5 = TX).

// enable port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

// Enable UART 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);

// configure GPIO pin as UART RX and TX pins
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

// setup UART 5 clock source as main system clock
    UARTClockSourceSet(UART5_BASE, UART_CLOCK_SYSTEM);

// register interrupt handler
    UARTIntRegister(UART5_BASE, BLUETOOTH_IntHandler);
    UARTIntEnable(UART5_BASE, UART_INT_RX);

// configure UART 5 module
    UARTConfigSetExpClk(
            UART5_BASE, SysCtlClockGet(), BLUETOOTH_BAUD_RATE,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

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

void BLUETOOTHprintf(char *string)
{

    char curChar = string[0];

    uint16_t count = 0;

    while (curChar != '\0')
    {
        UARTCharPut(UART5_BASE, curChar);
        count++;
        curChar = string[count];
    }

    return;
}

