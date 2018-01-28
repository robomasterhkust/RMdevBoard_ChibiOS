/**
 * Controlling shooter pwm
 */

#ifndef SHOOT_PWM_H_
#define SHOOT_PWM_H_

#include "pwm.h"

void pwm_shooter_init(void);

#endif /* SHOOT_PWM_H_ */
/*
 * shoot_pwm.h
 *
 *  Created on:
 *      Author: dell
 */

#ifndef SHOOT_PWM_H_
#define SHOOT_PWM_H_

void pwm_shooter_init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void pwm_config(PWMDriver *pwmp, const PWMConfig *config,int p);
void pwm12_config(PWMDriver *pwmp, const PWMConfig *config,int p);


#endif /* SHOOT_PWM_H_ */
