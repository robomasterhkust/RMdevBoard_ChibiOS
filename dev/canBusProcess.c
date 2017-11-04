/**
 * Edward ZHANG, 20171101
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 */
#include "ch.h"
#include "hal.h"

#include "canBusProcess.h"

#define GIMBAL_MOTOR_NUM  2U
#define CHASSIS_MOTOR_NUM 4U

static volatile GimbalEncoder_canStruct  gimbal_encoder[GIMBAL_MOTOR_NUM];
static volatile ChassisEncoder_canStruct chassis_encoder[CHASSIS_MOTOR_NUM];

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 * TODO
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, //HAL LIB, hcan1.Init.ABOM = DISABLE;hcan1.Init.AWUM = DISABLE;
  CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};

#define CAN_FILTER_NUM 28U
/* TODO */
static CANFilter canfilter[CAN_FILTER_NUM];

volatile GimbalEncoder_canStruct* can_getGimbalMotor(void)
{
  return gimbal_encoder;
}

volatile ChassisEncoder_canStruct* can_getChassisMotor(void)
{
  return chassis_encoder;
}

static inline void can_processChassisEncoder
  (ChassisEncoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
  chSysLock();
  cm->updated = true;
  cm->raw_angle = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
  cm->raw_speed = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
  chSysUnlock();
}

static inline void can_processGimbalEncoder
  (GimbalEncoder_canStruct* gm, const CANRxFrame* const rxmsg)
{
  chSysLock();
  gm->updated = true;
  gm->raw_angle        = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
  gm->raw_current      = (int16_t)((rxmsg->data8[2]) << 8 | rxmsg->data8[3]);
  gm->current_setpoint = (int16_t)((rxmsg->data8[4]) << 8 | rxmsg->data8[5]);
  chSysUnlock();
}

static void can_processEncoderMessage(const CANRxFrame* const rxmsg)
{
  switch(rxmsg->SID)
  {
      case CAN_CHASSIS_FL_FEEDBACK_MSG_ID:
        can_processChassisEncoder(&chassis_encoder[FRONT_LEFT] ,rxmsg);
        break;
      case CAN_CHASSIS_FR_FEEDBACK_MSG_ID:
        can_processChassisEncoder(&chassis_encoder[FRONT_RIGHT] ,rxmsg);
        break;
      case CAN_CHASSIS_BL_FEEDBACK_MSG_ID:
        can_processChassisEncoder(&chassis_encoder[BACK_LEFT] ,rxmsg);
        break;
      case CAN_CHASSIS_BR_FEEDBACK_MSG_ID:
        can_processChassisEncoder(&chassis_encoder[BACK_RIGHT] ,rxmsg);
        break;
      case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
        can_processGimbalEncoder(&gimbal_encoder[GIMBAL_YAW] ,rxmsg);
        break;
      case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
        can_processGimbalEncoder(&gimbal_encoder[GIMBAL_PITCH] ,rxmsg);
        break;
  }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {

  CANDriver* canp = (CANDriver*)p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX())
  {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(canp, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK)
    {
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

    txmsg.data8[0] = (uint8_t)(cm1_iq >> 8);
    txmsg.data8[1] = (uint8_t)cm1_iq;

    txmsg.data8[2] = (uint8_t)(cm2_iq >> 8);
    txmsg.data8[3] = (uint8_t)cm2_iq;

    txmsg.data8[4] = (uint8_t)(cm3_iq >> 8);
    txmsg.data8[5] = (uint8_t)cm3_iq;

    txmsg.data8[6] = (uint8_t)(cm4_iq >> 8);
    txmsg.data8[7] = (uint8_t)cm4_iq;

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void can_processInit(void)
{
  memset((void *)gimbal_encoder,  0, sizeof(GimbalEncoder_canStruct) *GIMBAL_MOTOR_NUM);
  memset((void *)chassis_encoder, 0, sizeof(ChassisEncoder_canStruct)*CHASSIS_MOTOR_NUM);

  uint8_t i;
  for (i = 0; i < CAN_FILTER_NUM; i++)
  {
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
                    can_rx, (void *)&CAND1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND2);

  chThdSleepMilliseconds(20);
}
