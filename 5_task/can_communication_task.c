//
// Created by beck on 5/4/2018.
//

#include <can_communication_task.h>
#include <can_lld.h>
#include "ch.h"
#include "hal.h"
#include "can_communication_task.h"

static volatile Gimbal_Send_Dbus_canStruct gimbal_send_dbus;
static volatile ROS_Msg_Struct ros_msg;

volatile Gimbal_Send_Dbus_canStruct* can_get_sent_dbus(void){
    return &gimbal_send_dbus;
}

volatile ROS_Msg_Struct* can_get_ros_msg(void) {
    return &ros_msg;
}

static inline void  can_processSendDbusEncoder
        (volatile Gimbal_Send_Dbus_canStruct* db, const CANRxFrame* const rxmsg)
{
    chSysLock();
    db->channel0 = rxmsg->data16[0];
    db->channel1 = rxmsg->data16[1];
    db->s1       = rxmsg->data8[4];
    db->s2       = rxmsg->data8[5];
    db->key_code = rxmsg->data16[3];
    chSysUnlock();
}

static inline void can_process_ros_command(volatile ROS_Msg_Struct * msg, const CANRxFrame* const rxmsg)
{
    chSysLock();
    msg->chassis_vx = (int16_t)rxmsg->data16[0];
    msg->chassis_vy = (int16_t)rxmsg->data16[1];
    msg->chassis_vw = (int16_t)rxmsg->data16[2];
    msg->pitch_vel_cmd = (int16_t)rxmsg->data16[3];
    msg->yaw_vel_cmd = (int16_t)rxmsg->data16[4];
    chSysUnlock();
}

/**
 * CAN bus sub function for decoding communication
 * @param rxmsg
 */
void can_process_communication(const CANRxFrame * const rxmsg)
{
    switch (rxmsg->SID)
    {
        case CAN_GIMBAL_BOARD_ID:
            can_processSendDbusEncoder(&gimbal_send_dbus,rxmsg);
            break;
        case CAN_CHASSIS_BOARD_ID:
            break;
        case CAN_NVIDIA_TX2_BOARD_ID:
            can_process_ros_command(&ros_msg, rxmsg);
            break;
        default: break;
    }
}