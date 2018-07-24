/*
 * barrelStatus.h
 *
 *  Created on: 30 Mar, 2018
 *      Author: ASUS
 */

#ifndef INC_BARRELSTATUS_H_
#define INC_BARRELSTATUS_H_

#include "stdint.h"
#include "stdbool.h"
#include "hal.h"
#include "string.h"
#include "canBusProcess.h"

#define INFANTRY


#define MM17
//#define 42MM

#define BARREL_UPDATE_FREQ 50
#define BARREL_UPDATE_PERIOD_US 1000000/BARREL_UPDATE_FREQ

#ifndef CAN_CHASSIS_SEND_BARREL_ID
  #define CAN_CHASSIS_SEND_BARREL_ID                  0x002
#endif

#ifdef INFANTRY
    #define HEATLIMIT_LVL_1 90
    #define HEATLIMIT_LVL_2 180
    #define HEATLIMIT_LVL_3 360
#endif

#ifdef SENTRY
  #define HEATLIMIT_LVL_1 360
  #define HEATLIMIT_LVL_2 360
  #define HEATLIMIT_LVL_3 360
#endif

#ifdef HERO
  #ifdef MM17
    #define HEATLIMIT_LVL_1 90
    #define HEATLIMIT_LVL_2 180
    #define HEATLIMIT_LVL_3 360
  #endif

  #ifdef MM42
    #define HEATLIMIT_LVL_1 90
    #define HEATLIMIT_LVL_2 180
    #define HEATLIMIT_LVL_3 360
  #endif
#endif

#ifdef AERIAL
  #define HEATLIMIT_LVL_1 3000
  #define HEATLIMIT_LVL_2 3000
  #define HEATLIMIT_LVL_3 3000
#endif

typedef enum{
  mm17,
  mm42
};

typedef struct {
  uint16_t heatLimit;
  uint16_t currentHeatValue;
  uint16_t remainHealth;
} barrelStatus_t, *pBarrelStatus;

pBarrelStatus barrelStatus_get(void);
void barrelStatus_init(void);
void updateBarrelStatus(void);
void barrelHeatLimitControl_init(void);



#endif /* INC_BARRELSTATUS_H_ */
