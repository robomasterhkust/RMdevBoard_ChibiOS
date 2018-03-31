/**
 * Edward ZHANG, 20171101
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 * @reference   RM2017_Archive
 */
#include <canBusProcess.h>
#include "ch.h"
#include "hal.h"

#include "canBusProcess.h"

static volatile GimbalEncoder_canStruct gimbal_encoder[GIMBAL_MOTOR_NUM];
static volatile ChassisEncoder_canStruct chassis_encoder[CHASSIS_MOTOR_NUM];
static volatile ChassisEncoder_canStruct extra_encoder[EXTRA_MOTOR_NUM];
static volatile Gimbal_Send_Dbus_canStruct gimbal_send_dbus;
static volatile UWB_canStruct uwb[UWB_NUM];

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */

bool uwb_start = false;
int uwb_index = 0;

static const CANConfig cancfg = {
        CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, //HAL LIB, hcan1.Init.ABOM = DISABLE;hcan1.Init.AWUM = DISABLE;
        CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
        CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};

#define CAN_FILTER_NUM 28U
static CANFilter canfilter[CAN_FILTER_NUM];

volatile GimbalEncoder_canStruct *can_getGimbalMotor(void) {
    return gimbal_encoder;
}

volatile ChassisEncoder_canStruct *can_getChassisMotor(void) {
    return chassis_encoder;
}

volatile ChassisEncoder_canStruct *can_getExtraMotor(void) {
    return extra_encoder;
}

volatile Gimbal_Send_Dbus_canStruct *can_get_sent_dbus(void) {
    return &gimbal_send_dbus;
}

UWB_canStruct *can_getUWB(void) {
    return uwb;
}

static inline void can_process_uwb(volatile UWB_canStruct *uwb_pointer, const CANRxFrame *const rxmsg) {
    chSysLock();
    uwb_pointer->x_world_cm = (int16_t) rxmsg->data16[0];
    uwb_pointer->y_world_cm = (int16_t) rxmsg->data16[1];
    uwb_pointer->theta_world_deg_100 = (uint16_t) rxmsg->data16[2];
    chSysUnlock();
}

static inline void can_processSendDbusEncoder
        (volatile Gimbal_Send_Dbus_canStruct *db, const CANRxFrame *const rxmsg) {
    chSysLock();
    db->channel0 = rxmsg->data16[0];
    db->channel1 = rxmsg->data16[1];
    db->s1 = rxmsg->data8[4];
    db->s2 = rxmsg->data8[5];
    db->key_code = rxmsg->data16[3];
    db->updated = true;
    chSysUnlock();
}

static inline void can_getMotorOffset
        (volatile ChassisEncoder_canStruct *cm, const CANRxFrame *const rxmsg) {
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
        (volatile ChassisEncoder_canStruct *cm, const CANRxFrame *const rxmsg) {
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
        (volatile GimbalEncoder_canStruct *gm, const CANRxFrame *const rxmsg) {
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

static void can_processEncoderMessage(CANDriver *const canp, const CANRxFrame *const rxmsg) {
    if (canp == &CAND1) {
        switch (rxmsg->SID) {
            case CAN_CHASSIS_FL_FEEDBACK_MSG_ID:
                chassis_encoder[FRONT_LEFT].msg_count++;
                chassis_encoder[FRONT_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_LEFT], rxmsg)
                                                            : can_processChassisEncoder(&chassis_encoder[FRONT_LEFT],
                                                                                        rxmsg);
                break;
            case CAN_CHASSIS_FR_FEEDBACK_MSG_ID:
                chassis_encoder[FRONT_RIGHT].msg_count++;
                chassis_encoder[FRONT_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[FRONT_RIGHT], rxmsg)
                                                             : can_processChassisEncoder(&chassis_encoder[FRONT_RIGHT],
                                                                                         rxmsg);
                break;
            case CAN_CHASSIS_BL_FEEDBACK_MSG_ID:
                chassis_encoder[BACK_LEFT].msg_count++;
                chassis_encoder[BACK_LEFT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_LEFT], rxmsg)
                                                           : can_processChassisEncoder(&chassis_encoder[BACK_LEFT],
                                                                                       rxmsg);
                break;
            case CAN_CHASSIS_BR_FEEDBACK_MSG_ID:
                chassis_encoder[BACK_RIGHT].msg_count++;
                chassis_encoder[BACK_RIGHT].msg_count <= 50 ? can_getMotorOffset(&chassis_encoder[BACK_RIGHT], rxmsg)
                                                            : can_processChassisEncoder(&chassis_encoder[BACK_RIGHT],
                                                                                        rxmsg);
                break;
            case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
                can_processGimbalEncoder(&gimbal_encoder[GIMBAL_YAW], rxmsg);
                break;
            case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
                can_processGimbalEncoder(&gimbal_encoder[GIMBAL_PITCH], rxmsg);
                break;
            case CAN_GIMBAL_SEND_DBUS_ID:
                can_processSendDbusEncoder(&gimbal_send_dbus, rxmsg);
        }
    } else {
        switch (rxmsg->SID) {
            case CAN_CHASSIS_FL_FEEDBACK_MSG_ID:
                can_processChassisEncoder(&extra_encoder[FRONT_LEFT], rxmsg);
                break;
            case CAN_CHASSIS_FR_FEEDBACK_MSG_ID:
                can_processChassisEncoder(&extra_encoder[FRONT_RIGHT], rxmsg);
                break;
            case CAN_CHASSIS_BL_FEEDBACK_MSG_ID:
                can_processChassisEncoder(&extra_encoder[BACK_LEFT], rxmsg);
                break;
            case CAN_CHASSIS_BR_FEEDBACK_MSG_ID:
                can_processChassisEncoder(&extra_encoder[BACK_RIGHT], rxmsg);
                break;
            case UWB_MSG_ID:
                if (rxmsg->DLC == 6 && !uwb_start) {
                    uwb_start = true;
                }
                if (uwb_start) {
                    if (rxmsg->DLC == 8) {
                        uwb_index++;
                    }
                    if (rxmsg->DLC == 6) {
                        uwb_index = 0;
                    }
                    if (uwb_index == 1) {
                        can_process_uwb(&uwb[0], rxmsg);
                    }
                }
                break;
        }
    }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);

static THD_FUNCTION(can_rx, p) {

    CANDriver *canp = (CANDriver *) p;
    event_listener_t el;
    CANRxFrame rxmsg;

    (void) p;
    chRegSetThreadName("can receiver");
    chEvtRegister(&canp->rxfull_event, &el, 0);
    while (!chThdShouldTerminateX()) {
        if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
            continue;
        while (canReceive(canp, CAN_ANY_MAILBOX,
                          &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
            can_processEncoderMessage(canp, &rxmsg);
        }
    }
    chEvtUnregister(&canp->rxfull_event, &el);
}

/*
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
                         const int16_t cm4_iq) {
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


void can_processInit(void) {
    memset((void *) gimbal_encoder, 0, sizeof(GimbalEncoder_canStruct) * GIMBAL_MOTOR_NUM);
    memset((void *) chassis_encoder, 0, sizeof(ChassisEncoder_canStruct) * CHASSIS_MOTOR_NUM);
    memset((void *) extra_encoder, 0, sizeof(ChassisEncoder_canStruct) * EXTRA_MOTOR_NUM);

    uint8_t i;
    for (i = 0; i < CAN_FILTER_NUM; i++) {
        canfilter[i].filter = i;
        canfilter[i].mode = 0; //CAN_FilterMode_IdMask
        canfilter[i].scale = 1; //CAN_FilterScale_32bit
        canfilter[i].assignment = 0;
        canfilter[i].register1 = 0;
        canfilter[i].register2 = 0;
    }

    canSTM32SetFilters(14, CAN_FILTER_NUM, canfilter);

    canStart(&CAND1, &cancfg);
    canStart(&CAND2, &cancfg);

    /*
     * Starting the transmitter and receiver threads.
     */
    chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
                      can_rx, (void *) &CAND1);
    chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                      can_rx, (void *) &CAND2);

    chThdSleepMilliseconds(20);
}

