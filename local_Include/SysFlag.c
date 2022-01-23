#include "SysFlag.h"

uint32_t g_uiSysFlags;


void SysFlag_Init( void )
{
    g_uiSysFlags = 0;
}

void SysFlag_Set( uint32_t uiSysFlag )
{
    BBA( &g_uiSysFlags, uiSysFlag ) = 1;
}

void SysFlag_Clear( uint32_t uiSysFlag )
{
    BBA( &g_uiSysFlags, uiSysFlag ) = 0;
}

bool SysFlag_Check( uint32_t uiSysFlag )
{
    return BBA( &g_uiSysFlags, uiSysFlag );
}
