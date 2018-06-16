/*
 * barrelStatus.c
 *
 *  Created on: 30 Mar, 2018
 *      Author: ASUS
 */
#include "barrel_heat_controller.h"

#ifndef CHASSIS
  #define CHASSIS
#endif

//#ifndef GIMBAL
//  #define GIMBAL
//#endif

#ifdef MM17
  uint8_t bulletType = mm17;
#endif

#ifdef MM42
  uint8_t bulletType = mm42;
#endif


static barrelStatus_t bStatus;
static uint16_t heatIncrement = 24;

pBarrelStatus barrelStatus_get(void){
  return &bStatus;
}

void barrelStatus_init(void){
  bStatus.heatLimit = HEATLIMIT_LVL_1;
  bStatus.firingStatus = 1;
  bStatus.currentHeatValue = 0;
  bStatus.updated = false;
}

void updateBarrelHeatStructure(judge_fb_t* jStruct){ // found in judge.c
  power_fb_t pInfo = (power_fb_t)(jStruct->powerInfo);
  game_fb_t gInfo = (game_fb_t)(jStruct->gameInfo);

  switch(gInfo.robotLevel){
    case 1:
      bStatus.heatLimit = HEATLIMIT_LVL_1;
      break;
    case 2:
      bStatus.heatLimit = HEATLIMIT_LVL_2;
      break;
    case 3:
      bStatus.heatLimit = HEATLIMIT_LVL_3;
      break;
  }

  if(bulletType == mm17){
    bStatus.currentHeatValue = pInfo.shooterHeat0;
  }else if(bulletType == mm42){
    bStatus.currentHeatValue = pInfo.shooterHeat1;
  }

  if(bStatus.currentHeatValue > bStatus.heatLimit){
    bStatus.firingStatus = CEASED_FIRE;
  }else{
    bStatus.firingStatus = CAN_FIRE;
  }
  bStatus.updated = true;
}

bool getBarrelStatus(void){
  #ifdef CHASSIS
    if(bStatus.updated == true){
      if(bStatus.firingStatus == CEASED_FIRE){
        bStatus.updated = false;
        return false;
      }else if(bStatus.firingStatus == CAN_FIRE){
        if(bStatus.currentHeatValue + heatIncrement < bStatus.heatLimit){
          bStatus.firingStatus = CAN_FIRE;
          bStatus.currentHeatValue += heatIncrement;
          bStatus.updated = false;
          return true;
        }else{
          bStatus.firingStatus = CEASED_FIRE;
          bStatus.updated = false;
          return false;
        }
      }
    }else{ // bStatus.update == false
      if(bStatus.firingStatus == CEASED_FIRE){
        return false;
      }else if(bStatus.firingStatus == CAN_FIRE){
        if(bStatus.currentHeatValue + heatIncrement < bStatus.heatLimit){
          bStatus.firingStatus = CAN_FIRE;
          bStatus.currentHeatValue += heatIncrement;
          return true;
        }else{
          bStatus.firingStatus = CEASED_FIRE;
          return false;
        }
      }
    }
  #endif

// #ifdef GIMBAL
//     BarrelStatus_canStruct* can_bStatus = can_get_sent_barrelStatus();
//     bStatus.heatLimit = can_bStatus->heatLimit;
//     bStatus.currentHeatValue = can_bStatus->currentHeatValue;
//     bStatus.firingStatus = can_bStatus->firingStatus;
// #endif
  return false;
}


void barrelHeatLimitControl_init(void){
  barrelStatus_init();
  // chThdCreateStatic(barrel_status_wa, sizeof(barrel_status_wa),
  //                          NORMALPRIO, barrel_status, NULL);
 }
