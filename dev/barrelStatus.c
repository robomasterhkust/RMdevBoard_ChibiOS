/*
 * barrelStatus.c
 *
 *  Created on: 30 Mar, 2018
 *      Author: ASUS
 */
#include "barrelStatus.h"
//#include "judge.h"

//#ifndef CHASSIS
//  #define CHASSIS
//#endif

#ifndef GIMBAL
  #define GIMBAL
#endif

#ifdef MM17
  uint8_t bulletType = mm17;
#endif

#ifdef MM42
  uint8_t bulletType = mm42;
#endif

static barrelStatus_t bStatus;
pBarrelStatus barrelStatus_get(void){
  return &bStatus;
}

void barrelStatus_init(void){
  bStatus.heatLimit = HEATLIMIT_LVL_1;
  bStatus.currentHeatValue = 0;
}

void updateBarrelStatus(void){
  #ifdef CHASSIS
    judge_fb_t jStruct = judgeDataGet();
    power_fb_t pInfo = (power_fb_t)(jStruct.powerInfo);
    game_fb_t gInfo = (game_fb_t)(jStruct.gameInfo);

    switch(gInfo.robotLevel){
      case 1:
        bStatus.heatLimit = HEATLIMIT_LVL_1;
        break;
      case 2:
        bStatus.heatLimit = HEATLIMIT_LVL_2;
        break;
      case 3:
        bStatus.heatLimit =HEATLIMIT_LVL_3;
        break;
    }

    if(bulletType == mm17){
      bStatus.currentHeatValue = pInfo.shooterHeat0;
    }else if(bulletType == mm42){
      bStatus.currentHeatValue = pInfo.shooterHeat1;
    }
  #endif

#ifdef GIMBAL
    BarrelStatus_canStruct* can_bStatus = can_get_sent_barrelStatus();
    bStatus.heatLimit = can_bStatus->heatLimit;
    bStatus.currentHeatValue = can_bStatus->currentHeatValue;
#endif
}

static inline void BarrelStatus_txCan(CANDriver *const CANx, const uint16_t SID)
{
  CANTxFrame txmsg;
  BarrelStatus_canStruct txCan;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = SID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();
  txCan.currentHeatValue = bStatus.currentHeatValue;
  txCan.heatLimit = bStatus.heatLimit;

  memcpy(&(txmsg.data8), &txCan ,8);
  chSysUnlock();

  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static THD_WORKING_AREA(barrel_status_wa, 2048);
static THD_FUNCTION(barrel_status, p)
{

  (void)p;
  chRegSetThreadName("barrelStatus");

  uint32_t tick = chVTGetSystemTimeX();
  while(!chThdShouldTerminateX())
  {
    tick += US2ST(BARREL_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();

    }
    updateBarrelStatus();
    #ifdef CHASSIS
      BarrelStatus_txCan(BARREL_CAN,CAN_CHASSIS_SEND_BARREL_ID);
    #endif

  }
}

void barrelHeatLimitControl_init(void){
  barrelStatus_init();
  chThdCreateStatic(barrel_status_wa, sizeof(barrel_status_wa),
                           NORMALPRIO, barrel_status, NULL);
 }
