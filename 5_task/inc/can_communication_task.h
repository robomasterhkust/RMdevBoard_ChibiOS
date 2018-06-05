//
// Created by beck on 5/4/2018.
//

#ifndef RM_CHIBIOS_CAN_COMMUNICATION_TASK_H
#define RM_CHIBIOS_CAN_COMMUNICATION_TASK_H

#include "hal.h"
#include "stdint.h"
#include "can_bus.h"
#include "dbus.h"
#include "judge.h"

#define COMM_CAN_BUS &CAND2

typedef struct{
    uint16_t channel0;
    uint16_t channel1;
    uint8_t  s1;
    uint8_t  s2;
    uint16_t key_code;
} Gimbal_Send_Dbus_canStruct;

typedef struct {
    float vx;
    float vy;
    float vz;
} ROS_Msg_Struct;

typedef struct{
    game_fb_t         gameInfo;
    hlth_fb_t         hlthInfo;
    projectile_fb_t   projectileInfo;
    power_fb_t        powerInfo;
    rfid_fb_t         rfidInfo;
    buffer_fb_t       bufferInfo;
    location_fb_t     locationInfo;
    game_over_fb_t    gameOverInfo;
} Chassis_Send_Judge_canStruct;


#ifdef __cplusplus
extern "C" {
#endif

volatile Gimbal_Send_Dbus_canStruct* can_get_sent_dbus(void);
volatile Chassis_Send_Judge_canStruct* can_get_sent_judge(void);
volatile ROS_Msg_Struct* can_get_ros_msg(void);
void can_process_communication(const CANRxFrame * rxmsg);
void RC_txCan(RC_Ctl_t* RC_Ctl, CANDriver *const CANx, const uint16_t SID);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_CAN_COMMUNICATION_TASK_H
