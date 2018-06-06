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

static volatile Chassis_Send_Judge_canStruct judge_from_chassis_board={
        .gameInfo.remainTime = 0,
        .gameInfo.gameStatus = 0,
        .gameInfo.robotLevel = 0,
        .gameInfo.remainHealth = 0,
        .gameInfo.fullHealth = 0,
        .hlthInfo.hitPos = 0,
        .hlthInfo.deltaReason = 0,
        .projectileInfo.bulletType = 0,
        .projectileInfo.bulletFreq = 0,
        .projectileInfo.bulletSpeed = 0,
        .powerInfo.volt = 0.0f,
        .powerInfo.current = 0.0f,
        .powerInfo.power = 0.0f,
        .powerInfo.powerBuffer = 0.0f,
        .powerInfo.shooterHeat0 = 0,
        .powerInfo.shooterHeat1 = 0,
        .rfidInfo.cardType = 0,
        .rfidInfo.cardIdx = 0,
        .gameOverInfo.winner = 0,
        .bufferInfo.powerUpType = 0,
        .bufferInfo.powerUpPercentage = 0,
        .locationInfo.x = 0.0f,
        .locationInfo.y = 0.0f,
        .locationInfo.z = 0.0f,
        .locationInfo.yaw = 0.0f

};


uint16_t JudgementDataRetrievedType[] = {
    CAN_CHASSIS_BOARD_GAMEINFO_ID,
    CAN_CHASSIS_BOARD_HLTH_ID,
    CAN_CHASSIS_BOARD_PROJECTILE_ID,
    CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID,
    CAN_CHASSIS_BOARD_VOLT_CURRENT_ID,
    CAN_CHASSIS_BOARD_SHOOTERHEAT_ID,
    CAN_CHASSIS_BOARD_RFID_ID,
    CAN_CHASSIS_BOARD_BUFFERINFO_ID,
    CAN_CHASSIS_BOARD_LOCATION_X_Y_ID,
    CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID
};

volatile Gimbal_Send_Dbus_canStruct* can_get_sent_dbus(void){
    return &dbus_from_gimbal_board;
}

volatile Chassis_Send_Judge_canStruct* can_get_sent_judge(void){
    return &judge_from_chassis_board;
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
    int16_t msg_vx = (int16_t)rxmsg->data16[0];
    int16_t msg_vy = (int16_t)rxmsg->data16[1];
    int16_t msg_vz = (int16_t)rxmsg->data16[2];
    msg->vx = msg_vx * 0.001;
    msg->vy = msg_vy * 0.001;
    msg->vz = msg_vz * 0.001;
    chSysUnlock();
}

static inline void can_processSendJudgeData(volatile Chassis_Send_Judge_canStruct * msg, const CANRxFrame* const rxmsg)
{
    chSysLock();
    switch(rxmsg->SID){
        case CAN_CHASSIS_BOARD_GAMEINFO_ID: msg->gameInfo.remainTime = (uint16_t)rxmsg->data16[0];
                    msg->gameInfo.gameStatus = (uint8_t)rxmsg->data8[2];
                    msg->gameInfo.robotLevel = (uint8_t)rxmsg->data8[3];
                    msg->gameInfo.remainHealth = (uint16_t)rxmsg->data16[2];
                    msg->gameInfo.fullHealth = (uint16_t)rxmsg->data16[3];
                    // if(judge_from_chassis_board.gameInfo.remainHealth >300){
                    //     LEDB_ON();
                    // }
                    // if(judge_from_chassis_board.gameInfo.remainHealth <= 300){
                    //     LEDB_OFF();
                    // }
                    break;
        case CAN_CHASSIS_BOARD_HLTH_ID: msg->hlthInfo.hitPos = (uint8_t)rxmsg->data8[0];
                    msg->hlthInfo.deltaReason = ((uint8_t)rxmsg->data8[1]);
                    break;
        case CAN_CHASSIS_BOARD_PROJECTILE_ID: msg->projectileInfo.bulletType = (uint8_t)rxmsg->data8[4];
                    msg->projectileInfo.bulletFreq = (uint8_t)rxmsg->data8[5];
                    msg->projectileInfo.bulletSpeed = (float)(rxmsg->data32[0]);//* 0.00001f;
                    break;
        case CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID: msg->powerInfo.power = (float)rxmsg->data32[0];//* 0.00001f;
                    msg->powerInfo.powerBuffer = (float)rxmsg->data32[1];//* 0.00001f;
                    // if(judge_from_chassis_board.powerInfo.power < 0.005){
                    //     LEDY_OFF();
                    // }else{
                    //     LEDY_ON();
                    // }
                    break;
        case CAN_CHASSIS_BOARD_VOLT_CURRENT_ID: msg->powerInfo.volt = (float)rxmsg->data32[0];//* 0.00001f;
                    msg->powerInfo.current = (float)rxmsg->data32[1];//* 0.00001f;
                    break;
        case CAN_CHASSIS_BOARD_SHOOTERHEAT_ID: msg->powerInfo.shooterHeat0 = (uint16_t)rxmsg->data16[0];
                    msg->powerInfo.shooterHeat1 = (uint16_t)rxmsg->data16[1];
                    break;
        case CAN_CHASSIS_BOARD_RFID_ID: msg->rfidInfo.cardType = (uint8_t)rxmsg->data8[0];
                    msg->rfidInfo.cardIdx = (uint8_t)rxmsg->data8[1];
                    break;
        case CAN_CHASSIS_BOARD_BUFFERINFO_ID: msg->bufferInfo.powerUpType = (uint8_t)rxmsg->data8[0];
                    msg->bufferInfo.powerUpPercentage = ((uint8_t)rxmsg->data8[1]);
                    break;
        case CAN_CHASSIS_BOARD_LOCATION_X_Y_ID: msg->locationInfo.x = (uint16_t)rxmsg->data16[0];//* 0.00001f;
                    msg->locationInfo.y = (uint16_t)rxmsg->data16[1];//* 0.00001f;
                    break;
        case CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID: msg->locationInfo.z = (uint16_t)rxmsg->data16[0];//* 0.00001f;
                    msg->locationInfo.yaw = (uint16_t)rxmsg->data16[1];//* 0.00001f;
                    break;
    }
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
        case CAN_CHASSIS_BOARD_GAMEINFO_ID:
        case CAN_CHASSIS_BOARD_HLTH_ID:
        case CAN_CHASSIS_BOARD_PROJECTILE_ID:
        case CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID:
        case CAN_CHASSIS_BOARD_VOLT_CURRENT_ID:
        case CAN_CHASSIS_BOARD_SHOOTERHEAT_ID:
        case CAN_CHASSIS_BOARD_RFID_ID:
        case CAN_CHASSIS_BOARD_BUFFERINFO_ID:
        case CAN_CHASSIS_BOARD_LOCATION_X_Y_ID:
        case CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID:
            can_processSendJudgeData(&judge_from_chassis_board,rxmsg);
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

void JudgeData_txCan(judge_fb_t* JudgeData, CANDriver *const CANx, const uint16_t SID)
{
    typedef struct game_info_t{
        uint16_t      remainTime;
        uint8_t       gameStatus;
        uint8_t       robotLevel;
        uint16_t      remainHealth;
        uint16_t      fullHealth;
    }__attribute__((packed)) game_fb_t;

    typedef struct hlth_info_t{
        uint8_t       hitPos : 4;
        uint8_t       deltaReason : 4;
    }__attribute__((packed)) hlth_fb_t;

    typedef struct projectile_fb_t{
        float         bulletSpeed;
        uint8_t       bulletType;
        uint8_t       bulletFreq;
    }__attribute__((packed)) projectile_fb_t;

    typedef struct power_fb_t2{
        float         volt;
        float         current;
    }__attribute__((packed)) power_fb_t2;

    typedef struct power_fb_t1{
        float         power;
        float         powerBuffer;
    }__attribute__((packed)) power_fb_t1;

    typedef struct power_fb_t3{
        uint16_t      shooterHeat0;
        uint16_t      shooterHeat1;
    }__attribute__((packed)) power_fb_t3;

    typedef struct rfid_fb_t{
        uint8_t       cardType;
        uint8_t       cardIdx;
    }__attribute__((packed)) rfid_fb_t;

    typedef struct buffer_fb_t{
        uint8_t       powerUpType;
        uint8_t       powerUpPercentage;
    }__attribute__((packed)) buffer_fb_t;

    typedef struct location_fb_t1{
        float         x;
        float         y;
    }__attribute__((packed)) location_fb_t1;

    typedef struct location_fb_t2{
        float         z;
        float         yaw;
    }__attribute__((packed)) location_fb_t2;

    CANTxFrame txmsg;

    int i;
    for(i = 0; i < 10; i++){
        // Chassis_Send_Judge_canStruct txCan;
        txmsg.IDE = CAN_IDE_STD;
        txmsg.SID = JudgementDataRetrievedType[i];
        txmsg.RTR = CAN_RTR_DATA;
        txmsg.DLC = 0x08;

        chSysLock();
        switch(JudgementDataRetrievedType[i]){
            case CAN_CHASSIS_BOARD_GAMEINFO_ID:{
                game_fb_t txCan1;
                txCan1.remainTime = JudgeData->gameInfo.remainTime;
                txCan1.gameStatus = JudgeData->gameInfo.gameStatus;
                txCan1.robotLevel = JudgeData->gameInfo.robotLevel;
                txCan1.remainHealth = JudgeData->gameInfo.remainHealth;
                txCan1.fullHealth = JudgeData->gameInfo.fullHealth;
                memcpy(&(txmsg.data8), &txCan1 ,8);
                break;
            }
            case CAN_CHASSIS_BOARD_HLTH_ID:{
                hlth_fb_t txCan2;
                txCan2.hitPos = JudgeData->hlthInfo.hitPos;
                txCan2.deltaReason = JudgeData->hlthInfo.deltaReason;
                memcpy(&(txmsg.data8), &txCan2, 8);
                break;
            }
            case CAN_CHASSIS_BOARD_PROJECTILE_ID:{
                projectile_fb_t txCan3;
                txCan3.bulletType = JudgeData->projectileInfo.bulletType;
                txCan3.bulletFreq = JudgeData->projectileInfo.bulletFreq;
                txCan3.bulletSpeed = JudgeData->projectileInfo.bulletSpeed;//* 0.00001f;
                memcpy(&(txmsg.data8), &txCan3,8);
                break;
            }
            case CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID:{
                power_fb_t1 txCan4;
                txCan4.power = JudgeData->powerInfo.power;//* 0.00001f;
                txCan4.powerBuffer = JudgeData->powerInfo.powerBuffer;//* 0.00001f;
                memcpy(&(txmsg.data8),&txCan4,8);
                break;
            }
            case CAN_CHASSIS_BOARD_VOLT_CURRENT_ID: {
                power_fb_t2 txCan5;
                txCan5.volt = JudgeData->powerInfo.volt;//* 0.00001f;
                txCan5.current = JudgeData->powerInfo.current;//* 0.00001f;
                memcpy(&(txmsg.data8), &txCan5, 8);
                break;
            }
            case CAN_CHASSIS_BOARD_SHOOTERHEAT_ID: {
                power_fb_t3 txCan6;
                txCan6.shooterHeat0 = JudgeData->powerInfo.shooterHeat0;
                txCan6.shooterHeat1 = JudgeData->powerInfo.shooterHeat1;
                memcpy(&(txmsg.data8), &txCan6,8);
                break;
            }
            case CAN_CHASSIS_BOARD_RFID_ID: {
                rfid_fb_t txCan7;
                txCan7.cardType = JudgeData->rfidInfo.cardType;
                txCan7.cardIdx = JudgeData->rfidInfo.cardIdx;
                memcpy(&(txmsg.data8), &txCan7,8);
                break;
            }
            case CAN_CHASSIS_BOARD_BUFFERINFO_ID:{
                buffer_fb_t txCan8;
                txCan8.powerUpType = JudgeData->bufferInfo.powerUpType;
                txCan8.powerUpPercentage = JudgeData->bufferInfo.powerUpPercentage;
                memcpy(&(txmsg.data8), &txCan8,8);
                break;
            }
            case CAN_CHASSIS_BOARD_LOCATION_X_Y_ID: {
                location_fb_t1 txCan9;
                txCan9.x = JudgeData->locationInfo.x;//* 0.00001f;
                txCan9.y = JudgeData->locationInfo.y;//* 0.00001f;
                memcpy(&(txmsg.data8), &txCan9,8);        
                break;
            }
            case CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID:{
                // LEDB_ON();
                location_fb_t2 txCan10;
                txCan10.z = JudgeData->locationInfo.z;//* 0.00001f;
                txCan10.yaw = JudgeData->locationInfo.yaw;//* 0.00001f;
                memcpy(&(txmsg.data8), &txCan10,8);
                break;
            }

        }

        chSysUnlock();
        canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
    }
}