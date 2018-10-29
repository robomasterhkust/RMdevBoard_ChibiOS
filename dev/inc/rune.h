#ifndef _RUNE_H_
#define _RUNE_H_

#define RUNE_FIRE_SAFE
#define RUNE_FIRE_POWER 160U
//#define RUNE_REMOTE_CONTROL         //turn this on when testing rune


#define RUNE_MAX_ERROR 0.03f
#define RUNE_THREAD_FREQUENCY 100U
#define RUNE_THREAD_PERIOD US2ST(1000000U/RUNE_THREAD_FREQUENCY)

#ifdef __cplusplus
extern "C" {
#endif
extern bool rune_remote_control_enable;
void rune_init(void);
void rune_cmd(uint8_t cmd);
void rune_fire(const float yaw, const float pitch, bool fire);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_RUNE_H_
