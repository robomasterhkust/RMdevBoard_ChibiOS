#ifndef _REMOTE_DEBUG_H_
#define _REMOTE_DEBUG_H_
#include <stdint.h>

#define REMOTE_DEBUG_CAN		&CAND2
#define CAN_EID_REMOTE_DEBUG 	0x003

void sendToDebuggerFloat(float*, float*);
void sendToDebuggerUINT8(uint8_t a,uint8_t b,uint8_t c,uint8_t d,uint8_t e,uint8_t f,uint8_t g,uint8_t h);
void sendToDebuggerUINT16(uint16_t a,uint16_t b,uint16_t c,uint16_t d);
void sendToDebuggerUINT32(uint32_t a,uint32_t b);

#endif