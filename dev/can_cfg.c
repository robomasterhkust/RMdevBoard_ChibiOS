/**
 * Edward ZHANG, 20171007
 * @file    can_cfg.c
 * @brief   CAN driver configuration file
 */
#include "ch.h"
#include "hal.h"

#include "can_cfg.h"

#define CAN_RXBUF_LEN 8U
typedef struct {
  CANDriver* const         canp;
  const uint32_t      tx_period;
} canConfigStruct;

static canConfigStruct can1 = {&CAND1, US2ST(1000000U/CAN1_TX_FREQ)};
static canConfigStruct can2 = {&CAND2, US2ST(1000000U/CAN2_TX_FREQ)};

/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
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

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);
static THD_FUNCTION(can_rx, p) {

  canConfigStruct* cip = (canConfigStruct*)p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&cip->canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX())
  {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(cip->canp, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK)
    {
      chSysLock();
      /* Process message.*/
      palTogglePad(GPIOF,GPIOF_LED_G);
      chSysUnlock();
    }
  }
  chEvtUnregister(&cip->canp->rxfull_event, &el);
}

/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can1_tx_wa, 256);
static THD_FUNCTION(can1_tx, p) {

  (void)p;
  CANTxFrame txmsg;

  chRegSetThreadName("can1 transmitter");

  /* TODO Modify this*/
  txmsg.IDE = CAN_IDE_STD;
  txmsg.EID = 0x200;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x33333333;
  txmsg.data32[1] = 0x33333333;

  uint32_t tick = chVTGetSystemTimeX();
  while (!chThdShouldTerminateX())
  {
    tick+= can1.tx_period;
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
  }
}

void RM_can_init(void)
{
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
                    can_rx, (void *)&can1);
  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                    can_rx, (void *)&can2);
  chThdCreateStatic(can1_tx_wa, sizeof(can1_tx_wa), NORMALPRIO + 7,
                    can1_tx, NULL);
}
