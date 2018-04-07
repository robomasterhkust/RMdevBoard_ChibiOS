//
// Created by beck on 5/4/2018.
//

#ifndef RM_CHIBIOS_CAN_UWB_TASK_H
#define RM_CHIBIOS_CAN_UWB_TASK_H

#define CAN_UWB_NUM           1U

#ifdef __cplusplus
extern "C" {
#endif

void can_process_uwb(const CANRxFrame * rxmsg);

typedef struct
{
    int16_t x_world_cm;
    int16_t y_world_cm;
    uint16_t theta;
} UWB_canStruct;

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_CAN_UWB_TASK_H
