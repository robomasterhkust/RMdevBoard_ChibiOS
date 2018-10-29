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

static BaseSequentialStream* chp = (BaseSequentialStream*)&SD3;

static inline void attitude_txcan(PIMUStruct const pIMU, CANDriver *const CANx, const uint16_t SID){
  CANTxFrame txmsg;
  Attitude_canStruct txCan;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = SID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  txCan.a = (int16_t) (pIMU->qIMU[0] * 1000.0f);
  txCan.b = (int16_t) (pIMU->qIMU[1] * 1000.0f);
  txCan.c = (int16_t) (pIMU->qIMU[2] * 1000.0f);
  txCan.d = (int16_t) (pIMU->qIMU[3] * 1000.0f);

  chSysLock();
  memcpy(&(txmsg.data8),&txCan,8);
  chSysUnlock();
  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

#define ATTITUDE_CAN_UPDATE_FREQ       100
#define ATTITUDE_CAN_UPDATE_PERIOD_US  1000000U/ATTITUDE_CAN_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_can_tx_thread_wa, 1024);
static THD_FUNCTION(Attitude_can_tx_thread, p)
{
  (void)p;
  PIMUStruct pIMU = imu_get();
  uint32_t tick = chVTGetSystemTimeX();

  while(!chThdShouldTerminateX())
  {
    tick += US2ST(ATTITUDE_CAN_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU->imu_Thd));
      chSysUnlock();
    }
    else
    {
      // attitude_txcan(pIMU, &CAND2, CAN_GIMBAL_SEND_ATTITUDE_ID);
    }
  }
}
#define attitude_can_init() (chThdCreateStatic(Attitude_can_tx_thread_wa, sizeof(Attitude_can_tx_thread_wa), \
                          NORMALPRIO, \
                          Attitude_can_tx_thread, NULL))

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  PIMUStruct pIMU = imu_get();
  PGyroStruct pGyro = gyro_get();

  static const IMUConfigStruct imu1_conf =
    {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_1000, MPU6500_AXIS_REV_X};
  imuInit(pIMU, &imu1_conf);

  imuGetData(pIMU);
  if(pIMU->temperature > 0.0f)
    tempControllerInit();
  else
    pIMU->errorCode |= IMU_TEMP_ERROR;

  attitude_imu_init(pIMU);

  uint32_t tick = chVTGetSystemTimeX();

  while(!chThdShouldTerminateX())
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
    attitude_update(pIMU, pGyro);

    // if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    // {
    //   chSysLock();
    //   chThdSuspendS(&(pIMU->imu_Thd));
    //   chSysUnlock();
    // }
  }
}

#define attitude_init() (chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa), \
                          NORMALPRIO + 5, \
                          Attitude_thread, NULL))

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg =
{
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000)
};

/*
 * Application entry point.
 */
int main(void)
{

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Init sequence 1: central controllers, loggers*/
  shellStart();
  params_init();

  //sdlog_init();
  extiinit();

  /* Init sequence 2: sensors, comm*/
  attitude_init();
  gyro_init();
  can_processInit();
  RC_init();
  barrelHeatLimitControl_init();
  attitude_can_init();

  // filter_init();
  can_comm_init();

  gimbal_init();
  feeder_init();

  while(!power_check())
  {
    LEDY_TOGGLE();
    chThdSleepMilliseconds(50);
  }
  LEDY_OFF();

  /* Init sequence 3: actuators, display*/
  gimbal_start();
  shooter_init();
  feeder_start();

  keyboardInit();
  #ifdef RUNE_REMOTE_CONTROL
    rune_init();
  #endif

  wdgStart(&WDGD1, &wdgcfg); //Start the watchdog

  while (true)
  {
    uint32_t error = gimbal_get_error();

    if(!power_failure())
    {
      wdgReset(&WDGD1);
    }
    else
      gimbal_kill();

    chThdSleepMilliseconds(200);
  }

  return 0;
}

/**
  *   @brief Check whether the 24V power is on
  */
bool power_check(void)
{
  GimbalEncoder_canStruct* can = can_getGimbalMotor();

  return can->updated;
}

/**
  *   @brief  Monitor the case of a failure on 24V power, indicating the vehicle being killed
  */
bool power_failure(void)
{
  uint32_t error = gimbal_get_error();

  return error & (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED);
}
