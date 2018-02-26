//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "ch.h"
#include "hal.h"
#include "shoot_pwm.h"

const int rc_max = 1864;
const int rc_min = 364;

int watch_width = 0;

/**
 * 2017/12/17 PWM test
 * @return
 */
int perc = 2600;

float map(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

  (int)p;
  float alpha = 0.2f;
  int y = 1050;
  int x = p;
  RC_Ctl_t* RC_Ctl = RC_get();
  //pwmEnableChannel(&PWMD12,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD12, x));
  //pwmEnableChannel(&PWMD12,1,PWM_PERCENTAGE_TO_WIDTH(&PWMD12, x));
  /*while(!chThdShouldTerminateX()){
    int width = (int)map(RC_Ctl->rc.channel1,rc_min,rc_max,1000,2600);
    watch_width = width;
    pwmEnableChannel(&PWMD12,0,PWM_PERCENTAGE_TO_WIDTH(&PWMD12, width));
    chThdSleepSeconds(1);
  }*/


  while(y<x){
    watch_width = y;
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

    //chThdSleepSeconds(3);

    pwm12_config(&PWMD12,&pwm12cfg,9000);
    //pwm12_config(&PWMD12,&pwm12cfg,9000,1);
    chThdSleepSeconds(2);

    pwm12_config(&PWMD12,&pwm12cfg,1000);
    //pwm12_config(&PWMD12,&pwm12cfg,1000,1);
    chThdSleepSeconds(2);
    pwm12_config(&PWMD12,&pwm12cfg,1200);
    //pwm12_config(&PWMD12,&pwm12cfg,1200,1);
    chThdSleepMilliseconds(1);

    chThdCreateStatic(pwm_accel_wa, sizeof(pwm_accel_wa),
      NORMALPRIO + 6, pwm_accel, 1500);
    chThdSleepMilliseconds(1);
}


