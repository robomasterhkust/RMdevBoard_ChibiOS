#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "canBusProcess.h"
#include "mpu6500.h"
#include "params.h"

#define GIMBAL_CONTROL_FREQ 1000U
#define GIMBAL_CUTOFF_FREQ    30U
//#define GIMBAL_ENCODER_USE_SPEED

#define GIMBAL_CAN  &CAND1
#define GIMBAL_CAN_EID  0x1FF

#ifdef GIMBAL_ENCODER_USE_SPEED
  #define GIMBAL_SPEED_BUFFER_LEN      10U
#endif

typedef enum {
  GIMBAL_YAW_NOT_CONNECTED = 1<<0,
  GIMBAL_PITCH_NOT_CONNECTED = 1<<1,
  GIMBAL_INITALIZATION_TIMEOUT = 1<<2,
  GIMBAL_CONTROL_LOSE_FRAME = 1<<31
} gimbal_error_t;

#define GIMBAL_ERROR_COUNT    3U
#define GIMBAL_WARNING_COUNT  1U
static const char gimbal_error_messages[][GIMBAL_ERROR_COUNT] =
{
  "E:Gimbal yaw not connected",
  "E:Gimbal pitch not connected",
  "E:Gimbal init timeout"
};

static const char gimbal_warning_messages[][GIMBAL_WARNING_COUNT] =
{
  "W:Gimbal control lose frame"
};


typedef struct{
  param_t kp;
  param_t ki;
} __attribute__((packed)) pi_controller_t;

typedef struct{
  param_t kp;
  param_t ki;
  param_t kd;
} __attribute__((packed)) pid_controller_t;

typedef struct{
  bool inited;
  volatile GimbalEncoder_canStruct* _encoder_can;
  volatile IMUStruct* _pIMU;

  uint32_t errorFlag;

  /* motor status */
  uint8_t yaw_wait_count;
  uint8_t pitch_wait_count;
  float yaw_angle;
  float pitch_angle;
  float yaw_current;
  float pitch_current;

  #ifdef GIMBAL_ENCODER_USE_SPEED
    float yaw_speed_enc;
    float pitch_speed_enc;
  #endif

  float yaw_speed;
  float pitch_speed;

  /* Controller part*/
  pi_controller_t yaw_vel;
  pi_controller_t pitch_vel;

  /*Mechanical parameters*/
  param_t axis_init_pos[2];
  param_t axis_ff_weight[2];

  /* TODO: control intermidiate current output (phase prediction)*/
  float yaw_iq_cmd;
  float pitch_iq_cmd;

  /* control output*/
  float yaw_iq_output;
  float pitch_iq_output;

}  GimbalStruct;

GimbalStruct* gimbal_get(void);
uint32_t gimbal_get_error(void);
void gimbal_init(void);

#endif
