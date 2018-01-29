//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "ch.h"
#include "hal.h"

/**
 * 2017/12/17 PWM test
 * @return
 */

static PWMConfig pwm8cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.       */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;

//    while (!chThdShouldTerminateX()) {
//        pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, (int)arg));
//        pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, (int)arg));
//        pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, (int)arg));
//        pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, (int)arg));
//        chThdSleepMilliseconds(1);
//    }
}

void pwm_shooter_init(void)
{

//    pwmStart(&PWMD8, &pwm8cfg); //*
//    pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000));
//    pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000));
//    pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000));
//    pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000));
//    chThdSleepMilliseconds(1000);
//    pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//    pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//    pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//    pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//    chThdSleepMilliseconds(1000);
//
//    /**
//     * Starts the PWM channel 0 using 20% duty cycle.
//     */
//    int percent = 2000;
//    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, percent);
//    chThdSleepMilliseconds(1);
}
