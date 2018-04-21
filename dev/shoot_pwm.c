/*
 * Written by Yitong Zhang and Tianze Li
 * hardcoded the timer 12 initialization
 */

#include "stdint.h"
#include "ch.h"
#include "hal.h"

#include "shoot.h"
#include "dbus.h"

#define MIN_SHOOT_SPEED 100U
#define MAX_SHOOT_SPEED 900U

RC_Ctl_t *rc;

static uint16_t speed_sp = 0;
static bool safe = false;

static PWMDriver PWMD12;

void pwm12_setWidth(uint16_t width)
{
    PWMD12.tim->CCR[0] = width;
    PWMD12.tim->CCR[1] = width;
}

/**
 * 2017/12/17 PWM test
 * @return
 */
void shooter_control(uint16_t setpoint)
{
    if (setpoint > MAX_SHOOT_SPEED)
        setpoint = MAX_SHOOT_SPEED;
    else if (setpoint < MIN_SHOOT_SPEED)
        setpoint = MIN_SHOOT_SPEED;

    if (safe || setpoint <= MIN_SHOOT_SPEED)
        speed_sp = setpoint;
}

static const PWMConfig pwm12cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.    width   */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(pwm_thd_wa, 512);

static THD_FUNCTION(pwm_thd, arg)
{
    (void) arg;

    const float alpha = 0.004f;
    float speed = 0;

    while (!chThdShouldTerminateX()) {
#ifdef SHOOTER_USE_RC
        switch (rc->rc.s2) {
            case RC_S_UP:
                shooter_control(175);
                break;
            case RC_S_MIDDLE:
                shooter_control(135);
                break;
            case RC_S_DOWN:
                safe = true;
                shooter_control(100);
                break;
            default:
                break;
        }
#endif

        speed = alpha * (float) speed_sp + (1 - alpha) * speed;
        pwm12_setWidth((uint16_t) speed);

        chThdSleepMilliseconds(5);
    }
}

static void pwm12_start(void)
{
    PWMD12.tim = STM32_TIM12;
    PWMD12.channels = 2;

    uint32_t psc;
    uint32_t ccer;
    rccEnableTIM12(FALSE);
    rccResetTIM12();

    PWMD12.clock = STM32_TIMCLK1;

    PWMD12.tim->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
                        STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;

    psc = (PWMD12.clock / pwm12cfg.frequency) - 1;

    PWMD12.tim->PSC  = psc;
    PWMD12.tim->ARR  = pwm12cfg.period - 1;
    PWMD12.tim->CR2  = pwm12cfg.cr2;
    PWMD12.period = pwm12cfg.period;

    ccer = 0;
    switch (pwm12cfg.channels[0].mode & PWM_OUTPUT_MASK) {
        case PWM_OUTPUT_ACTIVE_LOW:
            ccer |= STM32_TIM_CCER_CC1P;
        case PWM_OUTPUT_ACTIVE_HIGH:
            ccer |= STM32_TIM_CCER_CC1E;
        default:
            ;
    }
    switch (pwm12cfg.channels[1].mode & PWM_OUTPUT_MASK) {
        case PWM_OUTPUT_ACTIVE_LOW:
            ccer |= STM32_TIM_CCER_CC2P;
        case PWM_OUTPUT_ACTIVE_HIGH:
            ccer |= STM32_TIM_CCER_CC2E;

        default:
            ;
    }

    PWMD12.tim->CCER  = ccer;
    PWMD12.tim->SR    = 0;

    PWMD12.tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

    PWMD12.state = PWM_READY;
}

void shooter_init(void)
{
    rc = RC_get();
    pwm12_start();

#ifndef SHOOTER_SETUP
    pwm12_setWidth(900);
    chThdSleepSeconds(3);

    pwm12_setWidth(100);
    chThdSleepSeconds(3);

    chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
#endif
}
