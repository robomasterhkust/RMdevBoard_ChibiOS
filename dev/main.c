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
//    detect_error_task_init();
//    sdlog_init();
    extiinit();

    /* Init sequence 2: sensors, comm */
    attitude_estimator_init();
    // Initialize ADIS16265 single axial gyroscope
    // TODO: check if ADIS16265 exist
    single_axis_gyro_init_adis16265();
//    imu_init_adis16470();
    can_bus_init();
    RC_init();

    /* Init sequence 3: actuators, display */
//    command_mixer_init();
//    gimbal_simpler_controller_init();
    gimbal_init();
    chassis_init();
//    shooter_init();
    shooter_rm3508_init();
    feeder_init();
    bullet_count_task_init();

    LASER_ON();

    while (!chThdShouldTerminateX()) {
        chThdSleepMilliseconds(500);
        LEDR_TOGGLE();
    }
    return 0;
}