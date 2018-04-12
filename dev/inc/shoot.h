#ifndef _SHOOT_H_
#define _SHOOT_H_

#define SHOOTER_USE_RC
//#define SHOOTER_SETUP

void pwm12_setWidth(uint16_t width);

void shooter_control(uint16_t setpoint);
void shooter_init(void);

#endif
