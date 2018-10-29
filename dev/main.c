
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

    /*
     * Init sequence 1: central controller
     */
    shellStart();
    params_init();
    // extiinit();

    /*
     * Init sequence 2: sensors and communication
     */
    attitude_init();
    can_processInit();
    RC_init();
    judgeinit();

    /*
     * Init sequence 3: core functions
     */
    // chassis_init();
    command_mixer_init();
    chassis_task_init();
    can_comm_init();

    /*
     * Other chassis functions
     */
//    customData_init();
//    pwm_magazine_cover_init();
//    weightinit();
//    barrelHeatLimitControl_init();


    while (!chThdShouldTerminateX()) {
        LEDR_TOGGLE();

        // if (!power_failure()) {
        //     wdgReset(&WDGD1);
        // } else {
        //     gimbal_kill();
        // }
        chThdSleepMilliseconds(200);
    }
    return 0;
}
