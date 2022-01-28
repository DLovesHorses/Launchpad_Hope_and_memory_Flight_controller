/*
 * File         :   uart.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for uart.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-26
 */

#include "local_include/global.h"

// Defines

// Macros

void UART0_Init(void);

void UART0_STDIO_IntHandler(void);
void UART0_STDIO_Init(void);

void UARTdebug(char *component, char *file, char *routine, char *operation,
               char *status);

