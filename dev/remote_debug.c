#include "remote_debug.h"
#include "ch.h"
#include "hal.h"
#include <stdint.h>

#include "canBusProcess.h"

void sendToDebuggerFloat(float* f1, float* f2){
  CANTxFrame txmsg;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.EID = CAN_EID_REMOTE_DEBUG;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();
  memcpy( &txmsg.data32[0], f1, sizeof (float));
  memcpy( &txmsg.data32[1], f2, sizeof (float));
  // txmsg.data32[0] = f1;
  // txmsg.data32[1] = f2;
  chSysUnlock();
  canTransmit(REMOTE_DEBUG_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void sendToDebuggerUINT8(uint8_t int1,uint8_t int2,uint8_t int3,uint8_t int4,uint8_t int5,uint8_t int6,uint8_t int7,uint8_t int8){
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_EID_REMOTE_DEBUG;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data8[0] = int1;
    txmsg.data8[1] = int2;

    txmsg.data8[2] = int3;
    txmsg.data8[3] = int4;

    txmsg.data8[4] = int5;
    txmsg.data8[5] = int6;

    txmsg.data8[6] = int7;
    txmsg.data8[7] = int8;
    chSysUnlock();

    canTransmit(REMOTE_DEBUG_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

void sendToDebuggerUINT16(uint16_t int1,uint16_t int2,uint16_t int3,uint16_t int4){
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_EID_REMOTE_DEBUG;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data16[0] = int1;
    txmsg.data16[1] = int2;

    txmsg.data16[2] = int3;
    txmsg.data16[3] = int4;
    chSysUnlock();

    canTransmit(REMOTE_DEBUG_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}
void sendToDebuggerUINT32(uint32_t int1,uint32_t int2){
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = CAN_EID_REMOTE_DEBUG;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    txmsg.data32[0] = int1;
    txmsg.data32[1] = int2;
    chSysUnlock();

    canTransmit(REMOTE_DEBUG_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}