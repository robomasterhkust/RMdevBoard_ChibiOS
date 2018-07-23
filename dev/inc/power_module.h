#ifndef _POWER_MODULE_H_
#define _POWER_MODULE_H_
#include "stdint.h"
#include "stdbool.h"
#include "hal.h"

#define POWER_MODULE_CAN        &CAND1

#define CAN_POWER_MODULE_RECEIVER_ID                0x050
#define CAN_POWER_MODULE_SEND_MODE_ID               0x055   

typedef enum{
	PJUDGE = 1,
	BOOST = 2
} power_mode_t;

typedef enum{
	INFANTRY_T = 1,
	HERO_T = 2
} robot_type_t;

typedef struct {
	uint8_t power_mode;
	uint8_t robotType;
	int* bitmap_for_powerModule;
	bool Shift_press;
} powerModuleStruct_t;

typedef struct{
	uint8_t power_mode;
	uint8_t robotType;
} powerModule_canTransmitStruct;

void power_module_init();

#endif