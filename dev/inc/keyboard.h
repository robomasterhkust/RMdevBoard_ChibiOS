#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
#include "stdint.h"
#include "stdbool.h"
#include "hal.h"
#include "string.h"
/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X     Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W             0x0001      //bit 0
//#define S             0x0002
//#define A             0x0004
//#define D             0x0008
//#define SHIFT     0x0010
//#define CTRL      0x0020
//#define Q             0x0040
//#define E             0x0080
//#define R             0x0100
//#define F             0x0200
//#define G             0x0400
//#define Z             0x0800
//#define X             0x1000
//#define C             0x2000
//#define V             0x4000      //bit 15
//#define B             0x8000
/******************************************************/
#define KEY_V       14
#define KEY_C       13
#define KEY_X       12
#define KEY_Z       11
#define KEY_G       10
#define KEY_F       9
#define KEY_R       8
#define KEY_E       7
#define KEY_Q       6
#define KEY_CTRL    5
#define KEY_SHIFT   4
#define KEY_D       3
#define KEY_A       2
#define KEY_S       1
#define KEY_W       0

#define KEYBOARD_UPDATE_FREQ 500
#define KEYBOARD_UPDATE_PERIOD_US 1000000/KEYBOARD_UPDATE_FREQ

typedef enum
{
  NORMAL_MODE = 0,
  FAST_MODE,
  SLOW_MODE,
} kb_move_e;

typedef enum
{
  KEY_RELEASE = 0,
  KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG,
} kb_state_e;



typedef struct
{
  float vx;
  float vy;
  float vw;

  uint8_t twist_ctrl;
  uint8_t buff_ctrl;
  uint8_t track_ctrl;

  uint8_t kb_enable;

  uint16_t lk_cnt;
  uint16_t rk_cnt;

  kb_state_e lk_sta;
  kb_state_e rk_sta;

  kb_move_e move;

  uint16_t x_spd_limit;
  uint16_t y_spd_limit;

} kb_ctrl_t;

extern kb_ctrl_t km;
extern int bitmap[15];
//void keyboard_chassis_process(chassisStruct*,Gimbal_Send_Dbus_canStruct* );
void keyboard_reset();
void keyboardInit();
//bool keyboard_enable(Gimbal_Send_Dbus_canStruct* );
#endif
