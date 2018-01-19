/*
 * chassis.h
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "can.h"
#include "dbus.h"
#include "pid_chassis.h"

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

// DBUS MACRO

#define RPM_MAX    ((int16_t) 10000)              //
#define RPM_MIN    ((int16_t) -10000)              //

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x
// MATH definition
float map(float x, float in_min, float in_max, float out_min, float out_max);


void drive_init(void);
uint16_t* get_drive(void);
void chassis_init(void);
void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1);
#endif /* INC_CHASSIS_H_ */


