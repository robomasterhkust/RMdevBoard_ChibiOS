/*
 * hcsr04.c
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */
#include "hcsr04.h"
static float lastdistance = 0.0f;

static ICUConfig icucfg8 = {
  ICU_INPUT_ACTIVE_HIGH,
  ICU_TIM_FREQ,                                    /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  NULL,
  NULL,
  ICU_CHANNEL_1,
  0
};

void icuwidthcb(ICUDriver *icup) {

  icucnt_t width = icuGetWidthX(icup);
  lastdistance = (SPEED_OF_SOUND * width * M_TO_CM) / (ICU_TIM_FREQ * 2);
}

float* hcsr04_getDistance(void){
  return &lastdistance;
}

static THD_WORKING_AREA(ultrasonic_thread_wa, 1024);
static THD_FUNCTION(ultrasonic_thread,p)
{
  (void)p;
  chRegSetThreadName("Ultrasonic");
  icuStart(&ICUD8, &icucfg8);
  icuStartCapture(&ICUD8);
  icuEnableNotifications(&ICUD8);
  while(true){
      /* Triggering */
      palSetPad(GPIOB, GPIOB_ADC1_IN9);
      chThdSleepMicroseconds(10);
      palClearPad(GPIOB, GPIOB_ADC1_IN9);
      chThdSleepMilliseconds(100);
    }
}

void ultrasonic_init(void){
  chThdCreateStatic(ultrasonic_thread_wa, sizeof(ultrasonic_thread_wa), NORMALPRIO + 1, ultrasonic_thread, NULL);

}
