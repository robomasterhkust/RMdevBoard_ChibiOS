/*
 * chassis.h
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "can.h"

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

void chassis_init(void);

#endif /* INC_CHASSIS_H_ */
