#ifndef BULLET_TRACKER_TASK_H_
#define BULLET_TRACKER_TASK_H_

#include "stdint.h"
#include "stdbool.h"
#include "chsystypes.h"
#include "hal.h"

/*-------------------------- WEIGHT Channel Definition ------------------------*/

#define SERIAL_WEIGHT                &SD3

// #define WEIGHT_BUFFER_SIZE           SERIAL_BUFFERS_SIZE

#define WEIGHT_BUFFER_SIZE           4

/* ----------------------- BULLET TRACKER Channel Definition---------------------------- */

#define BULLET_TRACKER_BUFFER_SIZE             ((uint8_t)1)

// #define UART_BULLET_TRACKER                    &UARTD3

typedef struct
{
	uint8_t rxbuf[BULLET_TRACKER_BUFFER_SIZE];
	bool rx_start_flag;
	UARTDriver* uart;
	thread_reference_t thread_handler;
	struct{
		uint8_t bulletCount;
		float weight;
		float weightOffset;
	}bullet_tracker;
	bool inited;
}Bullet_Tracker_t;


#ifdef __cplusplus
extern "C" {
#endif

Bullet_Tracker_t* bulletTracker_get(void);
void weightdecode(void);

void weightinit(void);

// bool getWeightInitStatus(void);



// void bulletTracker_init(void);
// int* getBulletTrackerError(void);

#ifdef __cplusplus
}
#endif

#endif