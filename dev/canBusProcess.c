/**
 * Edward ZHANG, 20171101
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 * @reference   RM2017_Archive
 */
#include "ch.h"
#include "hal.h"

#include "barrelStatus.h"
#include "canBusProcess.h"
#include "halconf.h"

static volatile GimbalEncoder_canStruct gimbal_encoder[GIMBAL_MOTOR_NUM];
static volatile BarrelStatus_canStruct chassis_send_barrel;
static volatile ChassisEncoder_canStruct feeder_encoder;

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 * TODO
 */
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, // HAL LIB, hcan1.Init.ABOM =
                                                // DISABLE;hcan1.Init.AWUM =
                                                // DISABLE;
    CAN_BTR_SJW(0) | CAN_BTR_TS2(3) | CAN_BTR_TS1(8) | CAN_BTR_BRP(2)};

#define CAN_FILTER_NUM 28U
/* TODO */
static CANFilter canfilter[CAN_FILTER_NUM];

volatile GimbalEncoder_canStruct *can_getGimbalMotor(void) {
  return gimbal_encoder;
}

volatile ChassisEncoder_canStruct *can_getFeederMotor(void) {
  return &feeder_encoder;
}

volatile BarrelStatus_canStruct *can_get_sent_barrelStatus(void) {
  return &chassis_send_barrel;
}

static volatile Ros_msg_canStruct ros_msg = {
    .py = 0, .pz = 0, .vy = 0, .vz = 0};

static volatile Rune_canStruct rune_can = {
    .py = 0.0, .pz = 0.0, .updated = false};

volatile Ros_msg_canStruct *can_get_ros_msg(void) { return &ros_msg; }

volatile Rune_canStruct *can_get_rune(void) { return &rune_can; }

#define CAN_ENCODER_RADIAN_RATIO 7.669904e-4f // 2*M_PI / 0x2000
static inline void
can_processChassisEncoder(volatile ChassisEncoder_canStruct *cm,
                          const CANRxFrame *const rxmsg) {
  uint16_t prev_angle = cm->raw_angle;

  chSysLock();
  cm->updated = true;
  cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
  cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
  cm->act_current = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
  cm->temperature = (uint8_t)rxmsg->data8[6];

  if (cm->raw_angle - prev_angle > CAN_ENCODER_RANGE / 2)
    cm->round_count--;
  else if (cm->raw_angle - prev_angle < -CAN_ENCODER_RANGE / 2)
    cm->round_count++;

  cm->total_ecd = cm->round_count * CAN_ENCODER_RANGE + cm->raw_angle;
  cm->radian_angle = cm->total_ecd * CAN_ENCODER_RADIAN_RATIO;

  chSysUnlock();
}

static inline void
can_processGimbalEncoder(volatile GimbalEncoder_canStruct *gm,
                         const CANRxFrame *const rxmsg) {
  uint16_t prev_angle = gm->raw_angle;

  chSysLock();
  gm->updated = true;
  gm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
  gm->raw_current = (int16_t)((rxmsg->data8[2]) << 8 | rxmsg->data8[3]);
  gm->current_setpoint = (int16_t)((rxmsg->data8[4]) << 8 | rxmsg->data8[5]);

  if (gm->raw_angle - prev_angle > CAN_ENCODER_RANGE / 2)
    gm->round_count--;
  else if (gm->raw_angle - prev_angle < -CAN_ENCODER_RANGE / 2)
    gm->round_count++;

  gm->radian_angle =
      ((float)gm->round_count * CAN_ENCODER_RANGE + gm->raw_angle) *
      CAN_ENCODER_RADIAN_RATIO;

  chSysUnlock();
}

static inline void
can_processSendBarrelStatus(volatile BarrelStatus_canStruct *db,
                            const CANRxFrame *const rxmsg) {
  chSysLock();
  db->heatLimit = (uint16_t)(rxmsg->data16[0]);
  // db->heatLimit           = 1600;
  db->currentHeatValue = (uint16_t)(rxmsg->data16[1]);
  chSysUnlock();
}

static inline void can_process_ros_command(volatile Ros_msg_canStruct *msg,
                                           const CANRxFrame *const rxmsg) {
  chSysLock();
  int16_t msg_py = (int16_t)rxmsg->data16[0];
  int16_t msg_pz = (int16_t)rxmsg->data16[1];
  int16_t msg_vy = (int16_t)rxmsg->data16[2];
  int16_t msg_vz = (int16_t)rxmsg->data16[3];
  msg->py = msg_py * 0.001;
  msg->pz = msg_pz * 0.001;
  msg->vy = msg_vy * 0.001;
  msg->vz = msg_vz * 0.001;
  chSysUnlock();
}

static inline void can_process_rune(volatile Rune_canStruct *rune_can,
                                    const CANRxFrame *const rxmsg) {
  chSysLock();
  int16_t msg_py = (int16_t)rxmsg->data16[0];
  int16_t msg_pz = (int16_t)rxmsg->data16[1];

  rune_can->py = msg_py * 0.001;
  rune_can->pz = msg_pz * 0.001;
  rune_can->updated = true;
  chSysUnlock();
}

static void can_processEncoderMessage(const CANRxFrame *const rxmsg) {
  switch (rxmsg->SID) {
  case CAN_FEEDER_FEEDBACK_MSG_ID:
    can_processChassisEncoder(&feeder_encoder, rxmsg);
    break;
  case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
    can_processGimbalEncoder(&gimbal_encoder[GIMBAL_YAW], rxmsg);
    break;
  case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
    can_processGimbalEncoder(&gimbal_encoder[GIMBAL_PITCH], rxmsg);
    break;
  case CAN_CHASSIS_SEND_BARREL_ID:
    can_processSendBarrelStatus(&chassis_send_barrel, rxmsg);
    break;
  case CAN_NVIDIA_TX2_BOARD_ID:
    can_process_ros_command(&ros_msg, rxmsg);
    break;
  case CAN_RUNE:
    can_process_rune(&rune_can, rxmsg);
    break;
  }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {

  CANDriver *canp = (CANDriver *)p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&canp->rxfull_event, &el, 0);
  while (!chThdShouldTerminateX()) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(canp, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) ==
           MSG_OK) {
      can_processEncoderMessage(&rxmsg);
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
void can_motorSetCurrent(CANDriver *const CANx, const uint16_t EID,
                         const int16_t cm1_iq, const int16_t cm2_iq,
                         const int16_t cm3_iq, const int16_t cm4_iq) {
  CANTxFrame txmsg;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.EID = EID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();
  txmsg.data8[0] = (uint8_t)(cm1_iq >> 8);
  txmsg.data8[1] = (uint8_t)cm1_iq;

  txmsg.data8[2] = (uint8_t)(cm2_iq >> 8);
  txmsg.data8[3] = (uint8_t)cm2_iq;

  txmsg.data8[4] = (uint8_t)(cm3_iq >> 8);
  txmsg.data8[5] = (uint8_t)cm3_iq;

  txmsg.data8[6] = (uint8_t)(cm4_iq >> 8);
  txmsg.data8[7] = (uint8_t)cm4_iq;
  chSysUnlock();

  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void can_processInit(void) {
  memset((void *)gimbal_encoder, 0,
         sizeof(GimbalEncoder_canStruct) * GIMBAL_MOTOR_NUM);

#ifdef RM_CHASSIS_STANDARD
  memset((void *)chassis_encoder, 0,
         sizeof(ChassisEncoder_canStruct) * CHASSIS_MOTOR_NUM);
#endif

  uint8_t i;
  for (i = 0; i < CAN_FILTER_NUM; i++) {
    canfilter[i].filter = i;
    canfilter[i].mode = 0;  // CAN_FilterMode_IdMask
    canfilter[i].scale = 1; // CAN_FilterScale_32bit
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
  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7, can_rx,
                    (void *)&CAND1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7, can_rx,
                    (void *)&CAND2);

  chThdSleepMilliseconds(200);
}
