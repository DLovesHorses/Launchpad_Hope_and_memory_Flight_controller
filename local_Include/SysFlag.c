/*
 * File         :   SysFlag.c
 *
 * Description  :   Contains function to initialize and modify
 *                  system flags.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-24
 */

// Includes
#include "SysFlag.h"

// global variables and externs

uint32_t g_uiSysFlags;

// Function definitions.

void SysFlag_Init(void)
{
    g_uiSysFlags = 0;
}

void SysFlag_Set(uint32_t uiSysFlag)
{
    BBA( &g_uiSysFlags, uiSysFlag ) = 1;
}

void SysFlag_Clear(uint32_t uiSysFlag)
{
    BBA( &g_uiSysFlags, uiSysFlag ) = 0;
}

bool SysFlag_Check(uint32_t uiSysFlag)
{
    return BBA(&g_uiSysFlags, uiSysFlag);
}
