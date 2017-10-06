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

static BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_CMD;

static PIMUStruct pIMU;
static const IMUConfigStruct imu1_conf = {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_1000};

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");
  uint8_t errorCode;

  PIMUStruct pIMU_1 = (PIMUStruct)p;

  chThdSleepMilliseconds(100);
  errorCode = imuInit(pIMU_1, &imu1_conf);

  while(errorCode)
  {
    chprintf(chp,"IMU Init Failed: %d", errorCode);
    pIMU->data_invalid = true;
    chThdSleepMilliseconds(500);
  }

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(MPU6500_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    errorCode = imuGetData(pIMU_1);

    while(errorCode)
    {
      pIMU->data_invalid = true;
      chprintf(chp,"IMU Reading Error %d", errorCode);
      chThdSleepMilliseconds(500);
    }

    if(pIMU_1->accelerometer_not_calibrated || pIMU_1->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU_1->imu_Thd));
      chSysUnlock();
    }
  }
}
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
  RM_can_init();

//  tft_init(TFT_HORIZONTAL, CYAN, YELLOW, BLACK);

  pIMU = imu_get();

  chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa),
  NORMALPRIO + 5,
                    Attitude_thread, pIMU);

  palSetPad(GPIOE, GPIOE_LED_R);
  //palClearPad(GPIOF, GPIOF_LED_G);

  while (true)
  {
    palTogglePad(GPIOE, GPIOE_LED_R);
    //palTogglePad(GPIOF, GPIOF_LED_G);
    chThdSleepMilliseconds(500);
  }

  return 0;
}
