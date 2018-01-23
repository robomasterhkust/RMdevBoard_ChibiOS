/*
 * pid_chassis.c
 *
 *  Created on: Jan 15, 2018
 *      Author: luxiu
 */


#include "pid_chassis.h"
//
volatile PID CM1PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM2PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM3PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID CM4PID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT};
volatile PID HeadPID = { kP,kI,kD,0,0,0,0,0,{0},MAXCURRENT}; // might be Extra

//volatile PID CM1PID = { kP,kI,kD,0,0,0,0,0,1,MAXCURRENT,MAXINTEGRAL};
//volatile PID CM2PID = { kP,kI,kD,0,0,0,0,0,1,MAXCURRENT,MAXINTEGRAL};
//volatile PID CM3PID = { kP,kI,kD,0,0,0,0,0,1,MAXCURRENT,MAXINTEGRAL};
//volatile PID CM4PID = { kP,kI,kD,0,0,0,0,0,1,MAXCURRENT,MAXINTEGRAL};

//int16_t PID_output(int CM, int target_speed,volatile PID* PID){
//  //get the corresponding encoder
//  ChassisEncoder_canStruct* chassis_encoder_array = can_getChassisMotor();
//  ChassisEncoder_canStruct chassis_encoder = chassis_encoder_array[CM];
//  //update the data stored in encoder
//  PID->P = chassis_encoder.raw_speed + target_speed*GEARBOX_REDUCTION;
//  PID->D = PID->P - PID->LastError;
//  PID->I += PID->P;
//  PID->I = UpperLimit(PID->I,PID->I_Max);                                     //Limit I (+ve)
//  PID->I = LowerLimit(PID->I,-(PID->I_Max));
//  //set output and adjust it
//  PID->Output = (PID->P * PID->Kp) + (PID->I * PID->Ki) + (PID->D * PID->Kd); //Get PID
//  PID->Output = UpperLimit(PID->Output,PID->OutputLimit);
//  PID->Output = LowerLimit(PID->Output,-(PID->OutputLimit));
//  //some later steps
//  PID->LastError = PID->P;
//  PID->I = PID->I * PID->IntegralDecay;
//  return (int16_t)PID->Output;
//}

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
