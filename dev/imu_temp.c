/*
 * imu_temp.c
 *
 *  Created on: 24 Dec, 2017
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"
#include "mpu6500.h"

#define TEMP_THRESHOLD 38
#define TEMPERATURE_UPDATE_FREQ 100000U
#define TEMPERATURE_UPDATE_PERIOD_US 1000000U/TEMPERATURE_UPDATE_FREQ
static THD_WORKING_AREA(Temperature_thread_wa, 4096);
static THD_FUNCTION(Temperature_thread, p)
{
  chRegSetThreadName("IMU Temperature Control");

  (void)p;

  PIMUStruct pIMU = imu_get();

  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += US2ST(TEMPERATURE_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    imuGetData(pIMU);
    if(pIMU->temperature < TEMP_THRESHOLD){
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 9000));
    }else{
      pwmEnableChannel(&PWMD3, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
//      pwmDisableChannel(&PWMD3, 1);
    }
  }
}

