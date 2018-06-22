/*
 * shoot_pwm.h
 *
 *  Created on: 2018/1/9
 *      Author: Tom LI
 */

#ifndef SHOOT_PWM_H_
#define SHOOT_PWM_H_

//extern PWMDriver PWMD8;
extern PWMDriver PWMD12;

extern float map(float x, float in_min, float in_max, float out_min, float out_max);

void pwm_shooter_init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void pwm_config(PWMDriver *pwmp, const PWMConfig *config,int p);
void pwm12_config(PWMDriver *pwmp, const PWMConfig *config,int p);


#endif /* SHOOT_PWM_H_ */