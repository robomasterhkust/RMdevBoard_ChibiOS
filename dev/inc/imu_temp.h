/*
 * imu_temp.h
 *
 *  Created on: 7 Jan, 2018
 *      Author: ASUS
 */

#ifndef _IMU_TEMP_H_
#define _IMU_TEMP_H_

typedef struct tagTPIDStruct{
  float Kp;
  float Ki;
  float Kd;
  float Error_Integral;
  float Previous_Error;
  int PID_Value;
}__attribute__((packed)) TPIDStruct, *pTPIDStruct;

typedef struct {
  int Kp;
  int Ki;
  int Kd;
} TPIDConfigStruct;

#ifdef __cplusplus
extern "C" {
#endif

pTPIDStruct TPID_get(void);

#define tempController_kill() (pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0)))

void tempPID_Init(pTPIDStruct tempPID, const TPIDConfigStruct* const tpid_conf);
int tempPID_Update(pTPIDStruct tempPID, PIMUStruct pIMU);
void tempControllerInit(void);

#ifdef __cplusplus
#endif

#endif /* _IMU_TEMP_H_ */
