#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "canBusProcess.h"
#include "mpu6500.h"
#include "params.h"

#define GIMBAL_CONTROL_FREQ 1000U
#define GIMBAL_CUTOFF_FREQ    30U
//#define GIMBAL_ENCODER_USE_SPEED
#define GIMBAL_IQ_MAX 5000

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
#define GIMBAL_CONNECTION_ERROR_COUNT 20U
#define GIMBAL_CONTROL_PERIOD_NEXT    US2ST(1000000U/GIMBAL_CONTROL_FREQ)

static const char gimbal_error_messages[GIMBAL_ERROR_COUNT][50] =
{
  "E:Gimbal yaw not connected",
  "E:Gimbal pitch not connected",
  "E:Gimbal init timeout"
};

static const char gimbal_warning_messages[GIMBAL_WARNING_COUNT][50] =
{
  "W:Gimbal control lose frame"
};

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
    uint32_t timestamp;

  #ifdef GIMBAL_ENCODER_USE_SPEED
    float yaw_speed_enc;
    float pitch_speed_enc;
  #endif

  float yaw_atti_cmd;
  float pitch_atti_cmd;
  float yaw_speed_cmd;
  float pitch_speed_cmd;
  float yaw_speed;
  float pitch_speed;

  /*Mechanical parameters*/
  param_t axis_init_pos[3];
  param_t axis_ff_weight[6];
  /*first three subparams: pitch axis accelerometer maximum in XYZ
  last three subparams: yaw axis accelerometer maximum in XYZ when pitch at maximum*/
  param_t axis_ff_accel[6];

  /* TODO: control intermidiate current output (phase prediction)*/
  float yaw_iq_cmd;
  float pitch_iq_cmd;

  /* control output*/
  float yaw_iq_output;
  float pitch_iq_output;

}  GimbalStruct;

GimbalStruct* gimbal_get(void);
GimbalStruct* gimbal_get_sys_iden(void);
uint32_t gimbal_getError(void);
void gimbal_init(void);
void gimbal_sys_iden_init(void);

#endif
