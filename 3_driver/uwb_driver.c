//
// Created by beck on 5/4/2018.
//
#include "ch.h"
#include "hal.h"
#include "uwb_driver.h"

static volatile UWB_canStruct uwb[CAN_UWB_NUM];
static bool uwb_start = false;
static int uwb_index = 0;

static inline void can_decode_uwb(volatile UWB_canStruct *uwb_pointer, const CANRxFrame *const rxmsg) {
    chSysLock();
    uwb_pointer->x_world_cm = (int16_t) rxmsg->data16[0];
    uwb_pointer->y_world_cm = (int16_t) rxmsg->data16[1];
    uwb_pointer->theta = rxmsg->data16[2];
    chSysUnlock();
}

/**
 * CAN bus sub function to handle uwb sensor
 * @param rxmsg
 */
void can_process_uwb(const CANRxFrame * rxmsg)
{
    if (rxmsg->DLC == 6 && !uwb_start) {
        uwb_start = true;
    }
    if (uwb_start) {
        if (rxmsg->DLC == 8) {
            uwb_index++;
        }
        if (rxmsg->DLC == 6) {
            uwb_index = 0;
        }
        if (uwb_index == 1) {
            can_decode_uwb(&uwb[0], rxmsg);
        }
    }
}