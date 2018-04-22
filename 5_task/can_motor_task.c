//
// Created by beck on 5/4/2018.
//

#include "ch.h"
#include "hal.h"
#include "can_motor_task.h"

static volatile GimbalEncoder_canStruct gimbal_encoder[GIMBAL_MOTOR_NUM];
static volatile ChassisEncoder_canStruct chassis_encoder[CHASSIS_MOTOR_NUM];
static volatile ChassisEncoder_canStruct extra_encoder[EXTRA_MOTOR_NUM];

volatile GimbalEncoder_canStruct *can_getGimbalMotor(void)
{
    return gimbal_encoder;
}

volatile ChassisEncoder_canStruct *can_getChassisMotor(void)
{
    return chassis_encoder;
}

volatile ChassisEncoder_canStruct *can_getExtraMotor(void)
{
    return extra_encoder;
}

static inline void can_getMotorOffset
        (volatile ChassisEncoder_canStruct *cm, const CANRxFrame *const rxmsg)
{
    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t) (rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t) (rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t) (rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t) rxmsg->data8[6];
    chSysUnlock();

    cm->offset_raw_angle = cm->raw_angle;
}

static inline void can_processChassisEncoder
        (volatile ChassisEncoder_canStruct *cm, const CANRxFrame *const rxmsg)
{
    cm->last_raw_angle = cm->raw_angle;

    chSysLock();
    cm->updated = true;
    cm->raw_angle = (uint16_t) (rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    cm->raw_speed = (int16_t) (rxmsg->data8[2]) << 8 | rxmsg->data8[3];
    cm->act_current = (int16_t) (rxmsg->data8[4]) << 8 | rxmsg->data8[5];
    cm->temperature = (uint8_t) rxmsg->data8[6];
    chSysUnlock();

    if (cm->raw_angle - cm->last_raw_angle > CAN_ENCODER_RANGE / 2) cm->round_count--;
    else if (cm->raw_angle - cm->last_raw_angle < -CAN_ENCODER_RANGE / 2) cm->round_count++;

    cm->total_ecd = cm->round_count * CAN_ENCODER_RANGE + cm->raw_angle - cm->offset_raw_angle;
    cm->radian_angle = cm->total_ecd * CAN_ENCODER_RADIAN_RATIO;
}

static inline void can_processGimbalEncoder
        (volatile GimbalEncoder_canStruct *gm, const CANRxFrame *const rxmsg)
{
    gm->last_raw_angle = gm->raw_angle;

    chSysLock();
    gm->updated = true;
    gm->raw_angle = (uint16_t) (rxmsg->data8[0]) << 8 | rxmsg->data8[1];
    gm->raw_current = (int16_t) ((rxmsg->data8[2]) << 8 | rxmsg->data8[3]);
    gm->current_setpoint = (int16_t) ((rxmsg->data8[4]) << 8 | rxmsg->data8[5]);
    chSysUnlock();

    if (gm->raw_angle - gm->last_raw_angle > CAN_ENCODER_RANGE / 2) gm->round_count--;
    else if (gm->raw_angle - gm->last_raw_angle < -CAN_ENCODER_RANGE / 2) gm->round_count++;

    gm->total_ecd = gm->round_count * CAN_ENCODER_RANGE + gm->raw_angle - gm->offset_raw_angle;
    gm->radian_angle = gm->total_ecd * CAN_ENCODER_RADIAN_RATIO;
}

/**
 *  CAN bus sub function for decoding chassis encoder
 *  Chassis ID related encoder process, change the ID struct if hardware changed
 */
void can_process_chassis_encoder(const CANRxFrame * const rxmsg)
{
    switch (rxmsg->SID) {
        case CAN_CHASSIS_FL_FEEDBACK_MSG_ID:
            chassis_encoder[FRONT_LEFT].msg_count++;
            chassis_encoder[FRONT_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_LEFT], rxmsg)
                                                        : can_processChassisEncoder(&chassis_encoder[FRONT_LEFT], rxmsg);
            break;
        case CAN_CHASSIS_FR_FEEDBACK_MSG_ID:
            chassis_encoder[FRONT_RIGHT].msg_count++;
            chassis_encoder[FRONT_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_RIGHT], rxmsg)
                                                         : can_processChassisEncoder(&chassis_encoder[FRONT_RIGHT], rxmsg);
            break;
        case CAN_CHASSIS_BL_FEEDBACK_MSG_ID:
            chassis_encoder[BACK_LEFT].msg_count++;
            chassis_encoder[BACK_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_LEFT], rxmsg)
                                                       : can_processChassisEncoder(&chassis_encoder[BACK_LEFT], rxmsg);
            break;
        case CAN_CHASSIS_BR_FEEDBACK_MSG_ID:
            chassis_encoder[BACK_RIGHT].msg_count++;
            chassis_encoder[BACK_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_RIGHT], rxmsg)
                                                        : can_processChassisEncoder(&chassis_encoder[BACK_RIGHT], rxmsg);
            break;
        default: break;
    }
}

/**
 * CAN bus sub function for decoding extra motors, used mostly for hero
 * @param rxmsg
 */
void can_process_extra_encoder(const CANRxFrame * const rxmsg)
{
    uint16_t i = (uint16_t)rxmsg->SID - (uint16_t)CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_5;
    switch (rxmsg->SID) {
        case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_5:
        case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_6:
        case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_7:
        case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_8:
            extra_encoder[i].msg_count++;
            extra_encoder[i].msg_count <= 50 ? can_getMotorOffset(&extra_encoder[i], rxmsg)
                                             : can_processChassisEncoder(&extra_encoder[i], rxmsg);
            break;
        default: break;
    }
}

/**
 * CAN bus sub function for decoding gimbal motors
 * @param rxmsg
 */
void can_process_gimbal_encoder(const CANRxFrame * rxmsg)
{
    switch (rxmsg->SID) {
        case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
            can_processGimbalEncoder(&gimbal_encoder[GIMBAL_YAW], rxmsg);
            break;
        case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
            can_processGimbalEncoder(&gimbal_encoder[GIMBAL_PITCH], rxmsg);
            break;
        default:break;
    }
}

/**
 * @brief              Send motor current cmd using CAN driver
 * @param[in] cand     Pointer to CANDriver object we are currently using
 * @param[in] cmx_iq   Current (Torque) cmd of motor
 *
 * @notapi
 */
void can_motorSetCurrent(CANDriver *const CANx,
                         const uint16_t EID,
                         const int16_t cm1_iq,
                         const int16_t cm2_iq,
                         const int16_t cm3_iq,
                         const int16_t cm4_iq)
{
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = EID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data8[0] = (uint8_t) (cm1_iq >> 8);
    txmsg.data8[1] = (uint8_t) cm1_iq;

    txmsg.data8[2] = (uint8_t) (cm2_iq >> 8);
    txmsg.data8[3] = (uint8_t) cm2_iq;

    txmsg.data8[4] = (uint8_t) (cm3_iq >> 8);
    txmsg.data8[5] = (uint8_t) cm3_iq;

    txmsg.data8[6] = (uint8_t) (cm4_iq >> 8);
    txmsg.data8[7] = (uint8_t) cm4_iq;
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}
