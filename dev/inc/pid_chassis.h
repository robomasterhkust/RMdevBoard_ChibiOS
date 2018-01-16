/*
 * pid_chassis.h
 *
 *  Created on: Jan 13, 2018
 *      Author: luxiu
 */

#ifndef INC_PID_CHASSIS_H_
#define INC_PID_CHASSIS_H_


#include "canBusProcess.h"

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float LastError;
    float Output;
    float P;
    float I;
    float D;
    float speed_buffer[50];
    float OutputLimit;
} PID;

#define kP  5
#define kI  0.05
#define kD  2
#define MAXCURRENT	    16384

int16_t PID_output(int CM, int target_speed,volatile PID* PID);
#define UpperLimit(x,y)		( ((x) > (y)) ? (y) : (x) )		//return y if x>y, else return x
#define LowerLimit(x,y)		( ((x) < (y)) ? (y) : (x) )		//return y if x<y, else return x


#endif /* INC_PID_CHASSIS_H_ */
