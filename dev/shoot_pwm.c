//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "ch.h"
#include "hal.h"
#include "shoot_pwm.h"

/**
 * 2017/12/17 PWM test
 * @return
 */
       int perc = 2600;

extern float map(float x, float in_min, float in_max, float out_min, float out_max);

PWMConfig pwm8cfg = {
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

PWMConfig pwm12cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.       */
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

static THD_WORKING_AREA(pwm_accel_wa,512);
static THD_FUNCTION(pwm_accel,p){
  chRegSetThreadName("PWM acceleration");

  (void)p;
  float alpha = 0.1f;
  int y = 1100;
  int x = 2600;
  while(y<2600){
    y = alpha*x+(1-alpha)*y;
    pwmEnableChannel(&PWMD12,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD12, y));
    pwmEnableChannel(&PWMD12,1,PWM_PERCENTAGE_TO_WIDTH(&PWMD12, y));
    chThdSleepSeconds(1);
  }
}

void pwm_config(PWMDriver *pwmp, const PWMConfig *config,int p){
  pwmStop(pwmp);
  pwmStart(pwmp,config);
  pwmEnableChannel(pwmp, 0, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
  pwmEnableChannel(pwmp, 1, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
  pwmEnableChannel(pwmp, 2, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
  pwmEnableChannel(pwmp, 3, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
}

void pwm12_config(PWMDriver *pwmp, const PWMConfig *config,int p){
  pwmStop(pwmp);
  pwmStart(pwmp,config);
  pwmEnableChannel(pwmp, 0, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
  pwmEnableChannel(pwmp, 1, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
}


void pwm_shooter_init(void)
{
    LEDR_ON();
    LEDG_OFF();

    pwmStart(&PWMD12,&pwm12cfg);

    chThdSleepSeconds(3);
    //pwm_config(&PWMD8,&pwm8cfg,1000);
    pwm12_config(&PWMD12,&pwm12cfg,9000);
    chThdSleepSeconds(2);
    //pwm_config(&PWMD8,&pwm8cfg,1000);
    pwm12_config(&PWMD12,&pwm12cfg,1000);
    chThdSleepSeconds(2);
    pwm12_config(&PWMD12,&pwm12cfg,1100);
    chThdSleepMilliseconds(1);

    chThdCreateStatic(pwm_accel_wa, sizeof(pwm_accel_wa),
      NORMALPRIO + 6, pwm_accel, NULL);
    chThdSleepMilliseconds(1);
}

