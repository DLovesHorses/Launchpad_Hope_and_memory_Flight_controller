/*
 * File         :   <fileName>
 *
 * Description  :   Low Pass Filter code. First order filter (exponentially weighted moving average).
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */



// Includes

#include "LPF.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"


// global variables and externs



// Function definitions.
/*
 *
 *
 * see http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
 *
 * => New value will be stored in low_pass->prevOutput for next calculation.
 * => New value will also be returned
 * => input is the raw input of individual axis (e.g. : x-axis input of accelerometer.)
 * => dt is the time-step (h).
 */
float applyLPF(LOW_PASS_FILTER *low_pass, float input, float dt) {
    const float tau = 1.0f/(2.0f*PI*low_pass->Fc); // Time constant
    const float alpha = dt/(tau + dt);

    // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    float output = low_pass->prevOutput + alpha*(input - low_pass->prevOutput);
    low_pass->prevOutput = output;
    return output;
}
