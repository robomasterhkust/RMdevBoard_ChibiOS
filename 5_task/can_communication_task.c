//
// Created by beck on 5/4/2018.
//

#include "ch.h"
#include "hal.h"
#include "dbus.h"
#include "can_communication_task.h"
#include "string.h"

static volatile Gimbal_Send_Dbus_canStruct dbus_from_gimbal_board={
        .channel0 = RC_CH_VALUE_OFFSET,
        .channel1 = RC_CH_VALUE_OFFSET,
        .s1       = 0,
        .s2       = 0,
        .key_code = 0
};
static volatile ROS_Msg_Struct ros_msg={
        .vx = 0,
        .vy = 0,
        .vz = 0
};

volatile Gimbal_Send_Dbus_canStruct* can_get_sent_dbus(void){
    return &dbus_from_gimbal_board;
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
    msg->vx = (float)rxmsg->data16[0] * 0.00001f;
    msg->vy = (float)rxmsg->data16[1] * 0.00001f;
    msg->vz = (float)rxmsg->data16[2] * 0.00001f;
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
            can_processSendDbusEncoder(&dbus_from_gimbal_board,rxmsg);
            break;
        case CAN_CHASSIS_BOARD_ID:
            break;
        case CAN_NVIDIA_TX2_BOARD_ID:
            can_process_ros_command(&ros_msg, rxmsg);
            break;
        default: break;
    }
}

/**
 * CAN bus sub function for transmitting DBUS communication
 * @param RC_Ctl
 * @param CANx
 * @param SID
 */
void RC_txCan(RC_Ctl_t* RC_Ctl, CANDriver *const CANx, const uint16_t SID)
{
    CANTxFrame txmsg;
    Gimbal_Send_Dbus_canStruct txCan;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = SID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txCan.channel0 = RC_Ctl->rc.channel0;
    txCan.channel1 = RC_Ctl->rc.channel1;
    txCan.s1 = RC_Ctl->rc.s1;
    txCan.s2 = RC_Ctl->rc.s2;
    txCan.key_code = RC_Ctl->keyboard.key_code;

    memcpy(&(txmsg.data8), &txCan ,8);
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}