#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#include "can_bus.h"
#include "can_motor_task.h"
#include "mpu6500.h"
#include "params.h"

#define GIMBAL_CONTROL_FREQ 1000U
#define GIMBAL_CUTOFF_FREQ    30U

//#define GIMBAL_ZERO
//#define GIMBAL_INIT_TEST_PITCH    //Set Initialization position and PID value
//#define GIMBAL_INIT_TEST          //Set Initialization position and PID value
//#define GIMBAL_FF_TEST              //Set Initialization position and PID value
//#define GIMBAL_USE_MAVLINK_CMD
#define GIMBAL_USE_CAN_CMD
#define GIMBAL_ENCODER_USE_SPEED

#define GIMBAL_CAN  &CAND1
#define GIMBAL_CAN_EID  0x1FF

#define GIMBAL_YAW_GEAR 1.0f

typedef enum {
  GIMBAL_STATE_UNINIT = 0,
  GIMBAL_STATE_INITING,
  GIMBAL_STATE_READY,
  GIMBAL_STATE_FALLOFF,               //The vehicle is turned over
  GIMBAL_STATE_180DEG_TRANSITION,     //Reserved for sentry gimbal
  GIMBAL_YAW_AT_UP_LIMIT = 1 << 7,
  GIMBAL_YAW_AT_LOW_LIMIT = 1 << 6,
  GIMBAL_PITCH_AT_UP_LIMIT = 1 << 5,
  GIMBAL_PITCH_AT_LOW_LIMIT = 1 << 4,
} gimbal_state_t;

typedef enum {
  GIMBAL_YAW_NOT_CONNECTED = 1<<0,
  GIMBAL_PITCH_NOT_CONNECTED = 1<<1,
  GIMBAL_INITALIZATION_TIMEOUT = 1<<2,
  GIMBAL_CONTROL_LOSE_FRAME = 1<<31
} gimbal_error_t;

#define GIMBAL_ERROR_COUNT    3U
#define GIMBAL_WARNING_COUNT  1U
static const char *gimbal_error_messages[GIMBAL_ERROR_COUNT] =
{
  "E:Gimbal yaw not connected",
  "E:Gimbal pitch not connected",
  "E:Gimbal init timeout"
};

static const char gimbal_warning_messages[] =
{
  "W:Gimbal control lose frame"
};

typedef struct{
  uint8_t _wait_count;
  float _angle;
  float _current;

  #ifdef GIMBAL_ENCODER_USE_SPEED
    float _speed_enc;
    int8_t _dir;
  #endif

  float _speed_cmd;
  float _speed;
} GimbalMotorStruct;

typedef struct{
  uint8_t state;

  int32_t rev;
  float prev_yaw_cmd;
  uint32_t errorFlag;

  volatile IMUStruct* _pIMU;
  volatile GimbalEncoder_canStruct* _encoder;

  /* motor status */
  volatile GimbalMotorStruct motor[2];

  float yaw_atti_cmd;
  float pitch_atti_cmd;

  float chassis_yaw;
  float d_yaw;

  /*Mechanical parameters*/
  param_t axis_init_pos[4];
  param_t axis_ff_ext[6];
  param_t axis_ff_int[2];
  param_t axis_limit[4];

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
void gimbal_setRune(uint8_t cmd);
GimbalStruct* gimbal_get_sys_iden(void);
uint32_t gimbal_get_error(void);
void gimbal_init(void);
void gimbal_kill(void);

#endif
