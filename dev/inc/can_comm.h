//
// Created by beck on 29/10/18.
//

#ifndef RMDEVBOARD_CHIBIOS_CAN_COMM_H
#define RMDEVBOARD_CHIBIOS_CAN_COMM_H

#include "chassis.h"
#include "canBusProcess.h"

#define CAN_COMM_PORT &CAND2
#define CAN_CHASSIS_WHEEL_SID 0x216

#define CAN_COMM_FREQ 100U

#define CAN_COMM_FREQ_US     (1000000U/CAN_COMM_FREQ)
#define CAN_COMM_FREQ_ST     (US2ST(CAN_COMM_FREQ_US))

void can_transmit(CANDriver *const CANx, const uint16_t SID,
                  const int16_t val_0, const int16_t val_1,
                  const int16_t val_2, const int16_t val_3);

void can_comm_init(void);


#endif //RMDEVBOARD_CHIBIOS_CAN_COMM_H
