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

/* Mechanical design parameters */
#define GIMBAL_PITCH_ANGLE_OFFSET     0.0f
#define GIMBAL_YAW_ANGLE_OFFSET       0.0f

#ifdef GIMBAL_ENCODER_USE_SPEED
  #define GIMBAL_SPEED_BUFFER_LEN      10U
#endif

#define GIMBAL_ERROR_COUNT 3U
typedef enum {
  GIMBAL_YAW_NOT_CONNECTED = 1<<0,
  GIMBAL_PITCH_NOT_CONNECTED = 1<<1,
  GIMBAL_CONTROL_LOSE_FRAME = 1<<2
} gimbal_error_t;

static const char gimbal_error_messages[][GIMBAL_ERROR_COUNT] =
{
  "E:Gimbal yaw not connected",
  "E:Gimbal pitch not connected",
  "W:Gimbal control lose frame"
};

typedef struct{
  param_t kp;
  param_t ki;
} __attribute__((packed)) gimbal_vel_controller_t;

typedef struct{
  bool inited;
  GimbalEncoder_canStruct* _encoder_can;
  PIMUStruct _pIMU;

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
  param_t axis_ff_weight[2];
  gimbal_vel_controller_t yaw_vel;
  gimbal_vel_controller_t pitch_vel;

  /* TODO: control intermidiate current output (phase prediction)*/
  float yaw_iq_cmd;
  float pitch_iq_cmd;

  /* control output*/
  float yaw_iq_output;
  float pitch_iq_output;

}  GimbalStruct;

volatile GimbalStruct* gimbal_get(void);
uint32_t gimbal_get_error(void);
void gimbal_init(void);

#endif
