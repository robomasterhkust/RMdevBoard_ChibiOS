#ifndef CUSTOM_DATA_H_
#define CUSTOM_DATA_H_

#include "hal.h"

typedef struct{
	float data1;
	float data2;
	float data3;
	uint8_t lights8;
} Custom_Data_t;

void customData_init(void);

#endif