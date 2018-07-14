
/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  PIMUStruct pIMU = imu_get();

  static const IMUConfigStruct imu1_conf =
    {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_2000, MPU6500_AXIS_REV_NO};
  imuInit(pIMU, &imu1_conf);

  imuGetData(pIMU);
  if(pIMU->temperature > 0.0f)
    tempControllerInit();
  else
    pIMU->errorCode |= IMU_TEMP_ERROR;

  attitude_imu_init(pIMU);

  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += US2ST(MPU6500_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    if(pIMU->state == IMU_STATE_HEATING && pIMU->temperature > 61.0f)
      pIMU->state = IMU_STATE_READY;
    else if(pIMU->temperature < 55.0f || pIMU->temperature > 70.0f)
      pIMU->errorCode |= IMU_TEMP_WARNING;

    imuGetData(pIMU);
    attitude_update(pIMU);
  }
}

#define attitude_init() (chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa), \
                          NORMALPRIO + 5, \
                          Attitude_thread, NULL))

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellStart();
  params_init();
  //***
  // extiinit();
  //***
  can_processInit();
  pwm_magazine_cover_init();
  // magazineTracker_init();
  weightinit();
  RC_init();
  attitude_init();
  // pwm_shooter_init();
  judgeinit();
  barrelHeatLimitControl_init();
  // extiinit(); //*

  chassis_init();

  customData_init();

  while (true)
  {

    chThdSleepMilliseconds(500);

  }

  return 0;

  // while (!chThdShouldTerminateX()) {
  //     LEDR_TOGGLE();

  //     // if (!power_failure()) {
  //     //     wdgReset(&WDGD1);
  //     // } else {
  //     //     gimbal_kill();
  //     // }
  //     chThdSleepMilliseconds(200);
  // }
  // return 0;

}
