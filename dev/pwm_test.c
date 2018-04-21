/*
 * Created by beck on 21/4/2018.
 * Testing the TIM1, 4, 5, 8 PWM functions
 * TIM12 is harder to open and is located at shoot_pwm.c
 */
#include "ch.h"
#include "hal.h"
#include "pwm_test.h"

static PWMConfig pwm1_cfg_test = {
    100000,   /* 1MHz PWM clock frequency.   */
    100,      /* Initial PWM period 1ms.       */
    NULL,       /* Periodic call back */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* {<pwm_initialisation_status>, <callback function of the channel> */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
};

static PWMConfig pwm4_cfg_test = {
    100000,   /* 1MHz PWM clock frequency.   */
    100,      /* Initial PWM period 1ms.       */
    NULL,       /* Periodic call back */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* {<pwm_initialisation_status>, <callback function of the channel> */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
};

static PWMConfig pwm5_cfg_test = {
    100000,   /* 1MHz PWM clock frequency.   */
    100,      /* Initial PWM period 1ms.       */
    NULL,       /* Periodic call back */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* {<pwm_initialisation_status>, <callback function of the channel> */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
};

static PWMConfig pwm8_cfg_test = {
    100000,   /* 1MHz PWM clock frequency.   */
    100,      /* Initial PWM period 1ms.       */
    NULL,       /* Periodic call back */
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* {<pwm_initialisation_status>, <callback function of the channel> */
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL}
    },
    0,
    0
};

void test_init_all_pwm(void)
{
    pwmStart(&PWMD1, &pwm1_cfg_test);
    pwmStart(&PWMD4, &pwm4_cfg_test);
    pwmStart(&PWMD5, &pwm5_cfg_test);
    pwmStart(&PWMD8, &pwm8_cfg_test);
}

/*
 * Set pwm output in duty circle for four channels from 0 to 10000
 */
void set_pwm_to(PWMDriver *pwmp, int circle_1, int circle_2, int circle_3, int circle_4)
{
    pwmEnableChannel(pwmp, 0, PWM_PERCENTAGE_TO_WIDTH(pwmp, circle_1));
    pwmEnableChannel(pwmp, 1, PWM_PERCENTAGE_TO_WIDTH(pwmp, circle_2));
    pwmEnableChannel(pwmp, 2, PWM_PERCENTAGE_TO_WIDTH(pwmp, circle_3));
    pwmEnableChannel(pwmp, 3, PWM_PERCENTAGE_TO_WIDTH(pwmp, circle_4));
}
