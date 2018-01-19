/*
 * pid_chassis.c
 *
 *  Created on: Jan 15, 2018
 *      Author: luxiu
 */


#include "pid_chassis.h"

volatile PID CM1PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM2PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM3PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM4PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};

int16_t PID_output(int CM, int target_speed,volatile PID* PID){
	//get the corresponding encoder
	ChassisEncoder_canStruct* chassis_encoder_array = can_getChassisMotor();
	ChassisEncoder_canStruct chassis_encoder = chassis_encoder_array[CM];
	//update the data stored in encoder
	PID->P = -chassis_encoder.raw_speed + target_speed;
	PID->D = PID->P - PID->LastError;
	PID->I = 0;
	for (int i = 0 ; i<50 ; i++){
		PID->I += PID->speed_buffer[i];
	}
	//set output and adjust it
	PID->Output = PID->Kp * PID->P + PID->Ki * PID->I + PID->Kd * PID->D;
	PID->Output = UpperLimit(PID->Output,PID->OutputLimit);
	PID->Output = LowerLimit(PID->Output,-(PID->OutputLimit));
	//some later steps
	PID->LastError = PID->P;
	for (int i = 0; i < 49 ; i++){
		PID->speed_buffer[i] = PID->speed_buffer[i+1];
	}
	PID->speed_buffer[49] = PID->P;
	return (int16_t)PID->Output;
}


