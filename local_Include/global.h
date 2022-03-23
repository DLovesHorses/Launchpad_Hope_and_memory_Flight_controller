#ifndef GLOBAL_
#define GLOBAL_


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <math.h>


#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"

#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"

#include "utils/uartstdio.h"




#define ON true
#define OFF false

#define TRUE true
#define FALSE false

#define SAMPLE_TIME_MS     100     // 100 ms sample time


enum DeviceConnectionStatus{
    NOT_CONNECTED = 0,
    CONNECTED               // 1
};

enum DeviceInitializationStatus{
    NOT_INITIALIZED = 0,
    INITIALIZED
};


#endif /*GLOBAL_H */
