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

static THD_WORKING_AREA(Test_thread_wa, 128);
static THD_FUNCTION(Test_thread, p)
{
  (void)p;
  chRegSetThreadName("test");

  palSetPad(GPIOE, GPIOE_LED_R);
  palClearPad(GPIOF, GPIOF_LED_G);

  while(true)
  {
    palTogglePad(GPIOE, GPIOE_LED_R);
    palTogglePad(GPIOF, GPIOF_LED_G);
    chThdSleepMilliseconds(200);
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

  tft_init(TFT_HORIZONTAL, CYAN, YELLOW, BLACK);

  chThdCreateStatic(Test_thread_wa, sizeof(Test_thread_wa),
  NORMALPRIO - 10,
                    Test_thread, NULL);

  while (true)
  {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
