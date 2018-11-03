//
// Created by beck on 29/10/18.
//

#include "can_comm.h"

volatile chassis_t* chassis;

void
can_transmit(CANDriver *const CANx, const uint16_t SID,
             const int16_t val_0, const int16_t val_1,
             const int16_t val_2, const int16_t val_3)
{
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = SID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data8[0] = (uint8_t)(val_0 >> 8);
    txmsg.data8[1] = (uint8_t) val_0;

    txmsg.data8[2] = (uint8_t)(val_1 >> 8);
    txmsg.data8[3] = (uint8_t) val_1;

    txmsg.data8[4] = (uint8_t)(val_2 >> 8);
    txmsg.data8[5] = (uint8_t) val_2;

    txmsg.data8[6] = (uint8_t)(val_3 >> 8);
    txmsg.data8[7] = (uint8_t) val_3;
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static THD_WORKING_AREA(can_comm_thread, 512);

static THD_FUNCTION(can_comm_function, p)
{
    systime_t tick = chVTGetSystemTimeX();
    while (! chThdShouldTerminateX() ) {
        tick += CAN_COMM_FREQ_ST;
        if (tick > chVTGetSystemTimeX() )
            chThdSleepUntil(tick);
        else {
            tick = chVTGetSystemTimeX();
        }

        can_transmit(CAN_COMM_PORT, CAN_CHASSIS_WHEEL_SID,
                (int16_t)(chassis->_motors[0]._speed * 64.0f),
                (int16_t)(chassis->_motors[1]._speed * 64.0f),
                (int16_t)(chassis->_motors[2]._speed * 64.0f),
                (int16_t)(chassis->_motors[3]._speed * 64.0f));
    }
}


void
can_comm_init(void)
{
    chassis = chassis_struct_get();

    chThdCreateStatic(can_comm_thread, sizeof(can_comm_thread),
                      NORMALPRIO, can_comm_function, NULL);
}
