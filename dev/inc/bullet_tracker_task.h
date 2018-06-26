#ifndef BULLET_TRACKER_TASK_H_
#define BULLET_TRACKER_TASK_H_

#include "stdint.h"
#include "stdbool.h"
#include "chsystypes.h"
#include "hal.h"

/* ----------------------- WEIGHT Channel Definition---------------------------- */

#define BULLET_TRACKER_BUFFER_SIZE             ((uint8_t)1)

#define UART_BULLET_TRACKER                    &UARTD2

typedef struct
{
	uint8_t rxbuf[BULLET_TRACKER_BUFFER_SIZE];
	bool rx_start_flag;

	UARTDriver* uart;
	thread_reference_t thread_handler;
	struct{
		uint8_t bulletCount;
	}bullet_tracker;
}Bullet_Tracker_t;


#ifdef __cplusplus
extern "C" {
#endif

Bullet_Tracker_t* bulletTracker_get(void);
void bulletTracker_init(void);
int* getBulletTrackerError(void);

#ifdef __cplusplus
}
#endif

#endif