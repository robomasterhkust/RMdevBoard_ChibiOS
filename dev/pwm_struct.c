/*
 * pwm_struct.c
 *
 *  Created on: 22 Dec, 2017
 *      Author: ASUS
 */

#include "ch.h"
#include "hal.h"

//http://chibios.sourceforge.net/html/group___p_w_m.html#gaef1826611f5a65369e9e02d04e81ed61
// This is the link to the CHIBIos Documentation on PWMD. You can find all the necessary functions
// to command the PWM output in this webpage


//These are the templates for the callback functions
//Syntax:
//the name of this function means PWM 3 PERIODIC CALLBACK function
static void pwm3pcb(PWMDriver *pwmp) {

  (void)pwmp;

}

//the name of this function means PWM 3 CHANNEL 1 CALLBACK function
static void pwm3c1cb(PWMDriver *pwmp) {

  (void)pwmp;

}

// These are the definitions of the PWMConfig stuctures
// E.g. the function name means PWM 3 CONFIGURATION
static PWMConfig pwm3cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.       */
        pwm3pcb,       /* Periodic call back */
        {
                {PWM_OUTPUT_ACTIVE_HIGH, pwm3c1cb}, /* {<pwm_initialisation_status>, <callback function of the channel> */
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static PWMConfig pwm4cfg = {
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

void pwm3init(void){
  // Channel 1 is for the buzzer
  // Channel 2 is for the IMU heating element
  pwmStart(&PWMD3, &pwm3cfg);
}

void pwm4init(void){
  pwmStart(&PWMD4, &pwm4cfg);
  //#### these functions are use to enable the channel with certain pulse width.
  //#### the pulse width function can be found in the official documentation in the link above.
//  pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 1000));
//  pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, 1000));
}

void pwm8init(void){
  pwmStart(&PWMD8, &pwm8cfg);
//  pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//  pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//  pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
//  pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 9000));
}
