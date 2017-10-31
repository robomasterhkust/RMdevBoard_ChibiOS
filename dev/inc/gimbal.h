#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "canBusProcess.h"

#define GIMBAL_CONTROL_FREQ 1000U
#define GIMBAL_CUTOFF_FREQ    30U

#define GIMBAL_CAN  &CAND1
#define GIMBAL_CAN_EID  0x1FF

/* Mechanical design parameters */
#define GIMBAL_PITCH_ANGLE_OFFSET     0.0f
#define GIMBAL_YAW_ANGLE_OFFSET       0.0f

typedef enum {
  GIMBAL_YAW_NOT_CONNECTED = 1<<0,
  GIMBAL_PITCH_NOT_CONNECTED = 1<<1
  GIMBAL_CONTROL_LOSE_FRAME = 1<<2
} gimbal_error_t;

typedef struct{
  bool inited;
  GimbalEncoder_canStruct* encoder_can;
  uint32_t errorFlag;

  /* motor status */
  uint8_t yaw_wait_count;
  uint8_t pitch_wait_count;
  float yaw_angle;
  float pitch_angle;
  float yaw_current;
  float pitch_current;

  /* TODO: add FIR filter to encoder speed*/
  #ifdef GIMBAL_USE_SPEED
    float yaw_speed;
    float pitch_speed;
  #endif

  /* TODO: control intermidiate current output (phase prediction)*/
  float yaw_iq_cmd;
  float pitch_iq_cmd;

  /* control output*/
  float yaw_iq_output;
  float pitch_iq_output;

} GimbalStruct;

volatile GimbalStruct* gimbal_get(void);
void gimbal_init(void);

#endif
