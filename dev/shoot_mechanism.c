//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "pwm.h"
#include "pwm_lld.h"

extern PWMDriver PWMD2;

void pwm_init(void)
{
    PWMConfig pwm2cfg = {
        42000000,   // AHB Prescaler DIV1, APB1 prescalar DIV4
        42000,      // 1000 Hz
        NULL,
        {
            {PWM_OUTPUT_ACTIVE_HIGH, NULL},
            {PWM_OUTPUT_DISABLED, NULL},
            {PWM_OUTPUT_DISABLED, NULL},
            {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
    };
    pwmInit();
    pwmObjectInit(&PWMD2);
    pwmStart(&PWMD2, &pwm2cfg);
    pwmEnableChannel(&PWMD2, (pwmchannel_t)1U, (pwmcnt_t)1U);
}