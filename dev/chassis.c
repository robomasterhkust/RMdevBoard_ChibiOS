/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */
#include "chassis.h"

#define chassis_canUpdate()   \
  (can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    0, 0, 0, 0))

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
    (void)p;
    chRegSetThreadName("chassis controller");
    while(1){
      can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
          30000, 30000, 30000, 30000);
      chThdSleepMilliseconds(1000);
    }
}
void chassis_init(void){
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);
}

