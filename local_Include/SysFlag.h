/*
 * File         :   SysFlag.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for SysFlag.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-24
 */

// Includes
#include "local_include/global.h"

// Defines

// System flags (bit positions)
#define SYSFLAG_SYS_TICK       0
#define SYSFLAG_UART0_TX       1
#define SYSFLAG_UART0_RX       2

// Macros

// Bit-Band Alias
// a = address in the bit-band region:
//     0x20000000-0x200FFFFF
//     0x40000000-0x400FFFFF
// b = bit number:
//     0-31
#define BBA( a, b ) *( ( volatile uint32_t* )                       \
                    ( ( ( uint32_t )( a ) & 0x60000000 )            \
                    | 0x02000000                                    \
                    | ( ( ( ( uint32_t )( a ) & 0x000FFFFF ) << 5 ) \
                    + ( ( ( uint8_t )( b ) & 0x1F ) << 2 ) ) ) )

// Function prototypes
void SysFlag_Init(void);
void SysFlag_Set(uint32_t uiSysFlag);
void SysFlag_Clear(uint32_t uiSysFlag);
bool SysFlag_Check(uint32_t uiSysFlag);

