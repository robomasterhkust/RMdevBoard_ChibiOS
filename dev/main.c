/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, eit her express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

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

    // sdlog_init();
    extiinit();

    /* Init sequence 2: Power management */
    LASER_OFF();
//    POWER0_OFF();
//    POWER1_OFF();
//    POWER2_OFF();
//    POWER3_OFF();
//    LEDG1_ON();
    LEDR_ON();

    /* Init sequence 3: sensors, comm */
    // Initialize CAN bus receiver
    can_bus_init();
    attitude_estimator_init();
    magazineTracker_init();

#ifndef RM_CHASSIS_BOARD
    // Initialize ADIS16265 single axial gyroscope
    single_axis_gyro_init_adis16265();
    RC_init();

    while (!is_motor_power_on()) {
        // LEDG8_TOGGLE();
        LEDY_TOGGLE();
        chThdSleepMilliseconds(200);
    }
#else
    barrelHeatLimitControl_init();
    judgeinit();
#endif

    // test_init_all_pwm();

    /* Init sequence 4: actuators, display */
    // command_mixer_init();

#ifndef RM_CHASSIS_BOARD
    gimbal_init();
    shooter_init();
    feeder_init();
#else
    chassis_init();
#endif

    /* Init sequence 5: customized functions */
    detect_error_task_init();

    while (!chThdShouldTerminateX()) {
        LEDR_TOGGLE();

        if (!power_failure()) {
            wdgReset(&WDGD1);
        } else {
            gimbal_kill();
        }
        chThdSleepMilliseconds(200);
    }
    return 0;
}
