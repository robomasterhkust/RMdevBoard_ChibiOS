//
// Created by beck on 21/4/2018.
//

#ifndef RM_CHIBIOS_PWM_TEST_H
#define RM_CHIBIOS_PWM_TEST_H

#include "pwm.h"

void test_init_all_pwm(void);
void set_pwm_to(PWMDriver *pwmp, int circle_1, int circle_2, int circle_3, int circle_4);

#endif //RM_CHIBIOS_PWM_TEST_H
