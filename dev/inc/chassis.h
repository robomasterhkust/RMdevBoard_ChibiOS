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
#include "adis16265.h"

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

// DBUS MACRO

#define CURRENT_MAX    ((int16_t) 16384)              //
#define CURRENT_MIN    ((int16_t) -16384)              //

#define RPM_MAX    ((int16_t) 277)              //
#define RPM_MIN    ((int16_t) -277)              //

#define HEADING_MIN     ((float) -3.14159) // - pi
#define HEADING_MAX     ((float) 3.14159)   // pi
//#define HEADING_MIN     ((float) -1) // - pi
//#define HEADING_MAX     ((float) 1)   // pi
#define HEADING_SCALE   ((uint16_t) 1)

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x
// MATH definition
float map(float x, float in_min, float in_max, float out_min, float out_max);


void drive_init(void);
uint16_t* get_drive(void);
void chassis_init(void);
void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1);
#endif /* INC_CHASSIS_H_ */


