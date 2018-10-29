#ifndef _SHOOT_H_
#define _SHOOT_H_

#define SHOOTER_USE_RC
//#define SHOOTER_SETUP

void pwm12_setWidth(uint16_t width);

void shooter_control(uint16_t setpoint);
void shooter_init(void);

#if defined (RM_INFANTRY)
#define SHOOTER_CAN     &CAND1
#define SHOOTER_SID      0x005
#endif

typedef struct{
  uint8_t fast_speed;
  uint8_t slow_speed;
  uint8_t stop;
}speed_mode_t;

#endif
