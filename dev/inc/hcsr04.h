/*
 * hcsr04.h
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "main.h"

#define ICU_TIM_FREQ        1000000
#define M_TO_CM             100.0f
#define SPEED_OF_SOUND      343.2f

void icuwidthcb(ICUDriver *icup);
float* hcsr04_getDistance(void);


#endif /* INC_HCSR04_H_ */