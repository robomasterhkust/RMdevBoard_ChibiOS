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

//static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;
static const IMUConfigStruct imu1_conf =
        {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_250, MPU6500_AXIS_REV_Z};

static const magConfigStruct mag1_conf =
        {IST8310_ADDR_FLOATING, 200, IST8310_AXIS_REV_NO};

PIMUStruct pIMU;
PGyroStruct pGyro;

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);

static THD_FUNCTION(Attitude_thread, p)
{
    chRegSetThreadName("IMU Attitude Estimator");

    (void) p;

    imuInit(pIMU, &imu1_conf);
    ist8310_init(&mag1_conf);

    uint32_t tick = chVTGetSystemTimeX();

    while (true) {
        tick += US2ST(MPU6500_UPDATE_PERIOD_US);
        if (chVTGetSystemTimeX() < tick)
            chThdSleepUntil(tick);
        else {
            tick = chVTGetSystemTimeX();
            pIMU->errorCode |= IMU_LOSE_FRAME;
        }

        imuGetData(pIMU);
        ist8310_update();

        if (pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated) {
            chSysLock();
            chThdSuspendS(&(pIMU->imu_Thd));
            chSysUnlock();
        }
    }
}

mavlink_heartbeat_t packet_test = {
        963497464,
        17,
        84,
        151,
        218,
        3
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

    /* Init sequence 1: central controller, utility */
    shellStart();
    params_init();
//    sdlog_init();
//    extiinit();

    can_processInit();
    RC_init();


//    pGyro = gyro_init();
//    tempControllerInit(); //*

//    mavlinkComm_init();

//    chassis_init();
//    gimbal_init();
//    pwm_shooter_init(); // *
//    error_init();
//  pwm12init();
//
//  ultrasonic_init();

//    mavlinkComm_heartbeat_publish(&packet_test, 1);
//
//    mavlink_heartbeat_t* mavlink_rx = mavlinkComm_heartbeat_subscribe();

//  tft_init(TFT_HORIZONTAL, CYAN, YELLOW, BLACK);

    pIMU = imu_get();

    command_mixer_init();
    gimbal_simpler_controller_init();

//  chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa),
//                    NORMALPRIO + 5,
//                    Attitude_thread, NULL); //*



    while (!chThdShouldTerminateX()) {
//        chprintf(chp, "The usb shell chprintf still workings\n");
        chThdSleepMilliseconds(500);
        LEDR_TOGGLE();
    }

    return 0;
}