#include "magazine_cover_task.h"
#include "hal.h"

static PWMConfig pwm5cfg = {
        1000000,   /* 1MHz PWM clock frequency.   */
        20000,      /* Initial PWM period 20ms.       */
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

const int LEFTCOVER = 0; // D
const int RIGHTCOVER = 1; // C

void magCoverClose(void){
    pwmStop(&PWMD5);
    pwmStart(&PWMD5,&pwm5cfg);
    pwmEnableChannel(&PWMD5, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 5000));
    pwmEnableChannel(&PWMD5, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 5));
}

void magCoverOpen(void){
    pwmStop(&PWMD5);
    pwmStart(&PWMD5,&pwm5cfg);
    pwmEnableChannel(&PWMD5, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 500));
    pwmEnableChannel(&PWMD5, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 1000));
}

void pwm_magazine_cover_init(void)
{
    void pwm_config(PWMDriver *pwmp, const PWMConfig *config,int p){
        pwmStop(pwmp);
        pwmStart(pwmp,config);
        
        pwmEnableChannel(pwmp, 1, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
        pwmEnableChannel(pwmp, 2, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
        pwmEnableChannel(pwmp, 3, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
    }
    pwmStart(&PWMD5,&pwm5cfg);

    // chThdSleepSeconds(1);
}

