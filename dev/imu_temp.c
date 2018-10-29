/*
 * imu_temp.c
 *
 *  Created on: 3 Jan, 2017
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "mpu6500.h"
#include "imu_temp.h"

extern PWMDriver PWMD12;

#define TEMP_THRESHOLD 62
#define TEMPERATURE_UPDATE_PERIOD_S 1U

TPIDStruct tempPID1;

// These are the definitions of the PWMConfig stuctures
// E.g. the function name means PWM 3 CONFIGURATION
static PWMConfig pwm3cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        100,      /* Initial PWM period 1ms.       */
        NULL,       /* Periodic call back */
        {
                {PWM_OUTPUT_DISABLED, NULL}, /* {<pwm_initialisation_status>, <callback function of the channel> */
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static const TPIDConfigStruct tpid1_conf = {
                                            5600.0f,   //Kp
                                            70.0f,      //Ki
                                            0.0f       //Kd
};

pTPIDStruct TPID_get(void){
  return &tempPID1;
}

void tempPID_Init(pTPIDStruct tempPID, const TPIDConfigStruct* const tpid_conf){
  tempPID->Kp = tpid_conf->Kp;
  tempPID->Ki = tpid_conf->Ki;
  tempPID->Kd = tpid_conf->Kd;
  tempPID->Error_Integral = 0.0f;
  tempPID->Previous_Error = 0.0f;
  tempPID->PID_Value = 0;
}

int tempPID_Update(pTPIDStruct tempPID, PIMUStruct pIMU){
  float Error = TEMP_THRESHOLD - pIMU->temperature;
  tempPID->Error_Integral = tempPID->Error_Integral + Error;
  if(tempPID->Error_Integral > 2200)
    tempPID->Error_Integral = 2200;
  else if(tempPID->Error_Integral < -2200)
    tempPID->Error_Integral = -2200;

  float Error_Derivative = Error - tempPID->Previous_Error;
  tempPID->Previous_Error = Error;

  int PID_Output = (int)((tempPID->Kp * Error) + (tempPID->Ki * tempPID->Error_Integral) + (tempPID->Kd * Error_Derivative));

  if(PID_Output > 10000)
    PID_Output= 10000;
  else if(PID_Output < 0)
    PID_Output = 0;

  tempPID->PID_Value = PID_Output;

  return PID_Output;
}

static THD_WORKING_AREA(Temperature_thread_wa, 4096);
static THD_FUNCTION(Temperature_thread, p)
{
  chRegSetThreadName("IMU Temperature Control");

  (void)p;

  PIMUStruct pIMU = imu_get();
  pTPIDStruct tempPID = TPID_get();
  tempPID_Init(tempPID, &tpid1_conf);
  pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += S2ST(TEMPERATURE_UPDATE_PERIOD_S);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    if(pIMU->temperature > 88.0f)
    {
      pIMU->errorCode |= IMU_TEMP_ERROR;
      tempController_kill();
      chThdExit(MSG_OK);
    }

    int PWM_Output = tempPID_Update(tempPID, pIMU);
    pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, PWM_Output));
  }
}

void tempControllerInit(void){
  pwmStart(&PWMD3, &pwm3cfg);
  chThdCreateStatic(Temperature_thread_wa, sizeof(Temperature_thread_wa),
    NORMALPRIO - 10,
                      Temperature_thread, NULL);
}
