/**
 * Edward ZHANG, 20171101
 * @file    gimbal.c
 * @brief   Gimbal controller, driver and interface
 */
#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "math_misc.h"

#define GIMBAL_IQ_MAX 5000

#define gimbal_canUpdate()   \
  (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, \
    gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

static lpfilterStruct lp_yawAngle;
static lpfilterStruct lp_pitchAngle;
static GimbalStruct gimbal;

GimbalStruct* gimbal_get(void)
{
  return &gimbal;
}

uint32_t gimbal_get_error(void)
{
  return gimbal.errorFlag;
}

#ifdef GIMBAL_ENCODER_USE_SPEED
  static int16_t yaw_speed_buffer[GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t yaw_speed_count_buffer[GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t yaw_count_sum;
  static uint32_t yaw_speed_count;
  static int16_t pitch_speed_buffer[GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t pitch_speed_count_buffer[GIMBAL_SPEED_BUFFER_LEN];
  static uint8_t pitch_count_sum;
  static uint32_t pitch_speed_count;
#endif

#define GIMBAL_MAX_ANGLE 5.02655f
#define GIMBAL_MIN_ANGLE 1.25664f

#define GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define GIMBAL_CONNECTION_ERROR_COUNT 20U
static void gimbal_encoderUpdate(void)
{
  if(gimbal._encoder_can[GIMBAL_YAW].updated)
  {
    //Check validiaty of can connection
    gimbal._encoder_can[GIMBAL_YAW].updated = false;

    float angle_input = gimbal._encoder_can[GIMBAL_YAW].raw_angle*GIMBAL_ANGLE_PSC;

    //Prevent zero-crossing
    if(angle_input > GIMBAL_MAX_ANGLE && gimbal.yaw_angle < GIMBAL_MIN_ANGLE)
      angle_input -= 2*M_PI;
    else if(angle_input < GIMBAL_MIN_ANGLE && gimbal.yaw_angle > GIMBAL_MAX_ANGLE)
      angle_input += 2*M_PI;

    gimbal.yaw_angle = lpfilter_apply(&lp_yawAngle, angle_input);
    gimbal.yaw_current = gimbal._encoder_can[GIMBAL_YAW].raw_current;

    /* add FIR filter to encoder speed*/
    #ifdef GIMBAL_ENCODER_USE_SPEED
      yaw_count_sum += gimbal.yaw_wait_count;
      yaw_count_sum -= yaw_speed_count_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      int16_t diff = gimbal._encoder_can[GIMBAL_YAW].raw_angle -
        yaw_speed_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      //Detect zero-crossing scenerio
      if(diff > 6000)
        diff -= 8192;
      else if(diff < -6000)
        diff += 8192;

      gimbal.yaw_speed_enc = diff*GIMBAL_ANGLE_PSC * GIMBAL_CONTROL_FREQ/yaw_count_sum;
      yaw_speed_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal._encoder_can[GIMBAL_YAW].raw_angle;
      yaw_speed_count_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal.yaw_wait_count;
      yaw_speed_count++;
    #endif

    gimbal.yaw_wait_count = 1;
  }
  else
  {
    gimbal.yaw_wait_count++;
    if(gimbal.yaw_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
    {
      gimbal.errorFlag |= GIMBAL_YAW_NOT_CONNECTED;
      gimbal.yaw_wait_count = 1;
    }
  }

  if(gimbal._encoder_can[GIMBAL_PITCH].updated)
  {
    //Check validiaty of can connection
    gimbal._encoder_can[GIMBAL_PITCH].updated = false;

    float angle_input = gimbal._encoder_can[GIMBAL_PITCH].raw_angle*GIMBAL_ANGLE_PSC;

    //Prevent zero-crossing
    if(angle_input > GIMBAL_MAX_ANGLE && gimbal.pitch_angle < GIMBAL_MIN_ANGLE)
      angle_input -= 2*M_PI;
    else if(angle_input < GIMBAL_MIN_ANGLE && gimbal.pitch_angle > GIMBAL_MAX_ANGLE)
      angle_input += 2*M_PI;

    gimbal.pitch_angle = lpfilter_apply(&lp_pitchAngle, angle_input);
    gimbal.pitch_current = gimbal._encoder_can[GIMBAL_PITCH].raw_current;

    /* add FIR filter to encoder speed*/
    #ifdef GIMBAL_ENCODER_USE_SPEED
      pitch_count_sum += gimbal.pitch_wait_count;
      pitch_count_sum -= pitch_speed_count_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN];

      int16_t diff = gimbal._encoder_can[GIMBAL_PITCH].raw_angle -
        pitch_speed_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      //Detect zero-crossing scenerio
      if(diff > 4096)
        diff -= 8192;
      else if(diff < -4096)
        diff += 8192;

      gimbal.pitch_speed_enc = diff*GIMBAL_ANGLE_PSC * GIMBAL_CONTROL_FREQ/pitch_count_sum;
      pitch_speed_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal._encoder_can[GIMBAL_PITCH].raw_angle;
      pitch_speed_count_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal.pitch_wait_count;
      pitch_speed_count++;
    #endif

    gimbal.pitch_wait_count = 1;
  }
  else
  {
    gimbal.pitch_wait_count++;
    if(gimbal.pitch_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
    {
      gimbal.errorFlag |= GIMBAL_PITCH_NOT_CONNECTED;
      gimbal.pitch_wait_count = 1;
    }
  }
}

/*
 *  @brief      Utilize the gimbal dynamic model and add a feed-forward output
 *  @NOTE       Requires accelerometer raw data
 *
 *  @not-api
 */
static void gimbal_addFF(void)
{
  gimbal.yaw_iq_cmd = gimbal.axis_ff_weight[GIMBAL_YAW];
  gimbal.pitch_iq_cmd = gimbal.axis_ff_weight[GIMBAL_PITCH];
}


static void gimbal_getSpeed(void)
{
  gimbal.pitch_speed = gimbal._pIMU->gyroData[Pitch];
}

#define GIMBAL_CONTROL_PERIOD_ST     US2ST(1000000U/GIMBAL_CONTROL_FREQ)
static THD_WORKING_AREA(gimbal_thread_wa, 2048);
static THD_FUNCTION(gimbal_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal controller");

  uint32_t tick = chVTGetSystemTimeX();
  while(!chThdShouldTerminateX())
  {
    tick += GIMBAL_CONTROL_PERIOD_ST;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      gimbal.errorFlag |= GIMBAL_CONTROL_LOSE_FRAME;
    }

    gimbal_encoderUpdate();
    gimbal_getSpeed();

    gimbal_addFF();

    if(gimbal.yaw_iq_cmd < GIMBAL_IQ_MAX && gimbal.yaw_iq_cmd > -GIMBAL_IQ_MAX)
      gimbal.yaw_iq_output = gimbal.yaw_iq_cmd;

    if(gimbal.pitch_iq_cmd < GIMBAL_IQ_MAX && gimbal.pitch_iq_cmd > -GIMBAL_IQ_MAX)
      gimbal.pitch_iq_output = gimbal.pitch_iq_cmd;

    gimbal_canUpdate();
  }
}

static THD_WORKING_AREA(gimbal_init_thread_wa, 2048);
static THD_FUNCTION(gimbal_init_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal init thread");

  float _error[2];
  float _error_diff[2];
  float _error_int[2] = {0.0f, 0.0f};
  const float _error_int_max[2] = {1000.0f, 1500.0f};
  const float _output_max = 2500.0f;

  #ifndef ENCODER_USE_SPEED
  #define GIMBAL_POS_BUFFER_LEN 50U
    float _error_past[GIMBAL_POS_BUFFER_LEN][2];
    uint8_t i;
    for (i = 0; i < GIMBAL_POS_BUFFER_LEN; i++)
    {
      _error_past[i][0] = 0.0f;
      _error_past[i][1] = 0.0f;
    }
    uint8_t _error_index = 0, _prev_index = 1;
  #else
  #endif

  pid_controller_t _yaw_pos;
  pid_controller_t _pitch_pos;

  const char yaw_pos_name[] =   "Gimbal Yaw Pos";
  const char pitch_pos_name[] = "Gimbal Pitch Pos";

  params_set(&_yaw_pos,   3, 3, yaw_pos_name,   subname_PID,   PARAM_PUBLIC);
  params_set(&_pitch_pos, 4, 3, pitch_pos_name, subname_PID,   PARAM_PUBLIC);

  while(true)
  {
    gimbal_encoderUpdate();

    _error[GIMBAL_YAW] = gimbal.axis_init_pos[GIMBAL_YAW] - gimbal.yaw_angle;
    _error[GIMBAL_PITCH] = gimbal.axis_init_pos[GIMBAL_PITCH] - gimbal.pitch_angle;

    _error_int[GIMBAL_YAW] += _error[GIMBAL_YAW] * _yaw_pos.ki;
    _error_int[GIMBAL_PITCH] += _error[GIMBAL_PITCH] * _pitch_pos.ki;

    _prev_index = (_error_index + 1) % GIMBAL_POS_BUFFER_LEN;

    _error_past[_error_index][GIMBAL_PITCH] = _error[GIMBAL_PITCH];
    _error_past[_error_index][GIMBAL_YAW] = _error[GIMBAL_YAW];

    _error_diff[GIMBAL_PITCH] = _error[GIMBAL_PITCH] - _error_past[_prev_index][GIMBAL_PITCH];
    _error_diff[GIMBAL_YAW]   = _error[GIMBAL_YAW]   - _error_past[_prev_index][GIMBAL_YAW];
    _error_index = _prev_index;

    uint8_t i;
    for (i = 0; i < 2; i++)
    {
      if(_error_int[i] > _error_int_max[i])
        _error_int[i] = _error_int_max[i];
      else if(_error_int[i] < -_error_int_max[i])
        _error_int[i] = -_error_int_max[i];
    }

    gimbal.yaw_iq_output =
      _error[GIMBAL_YAW]*_yaw_pos.kp  + _error_int[GIMBAL_YAW] + _error_diff[GIMBAL_YAW]*_yaw_pos.kd;
    gimbal.pitch_iq_output =
      _error[GIMBAL_PITCH]*_pitch_pos.kp + _error_int[GIMBAL_PITCH] + _error_diff[GIMBAL_PITCH]*_pitch_pos.kd;

    if(gimbal.yaw_iq_output > _output_max)
      gimbal.yaw_iq_output = _output_max;
    else if(gimbal.yaw_iq_output < -_output_max)
      gimbal.yaw_iq_output = -_output_max;

    if(gimbal.pitch_iq_output > _output_max)
      gimbal.pitch_iq_output = _output_max;
    else if(gimbal.pitch_iq_output < -_output_max)
      gimbal.pitch_iq_output = -_output_max;

    gimbal_canUpdate();
    chThdSleepMilliseconds(1);
  }
}

/* name of controller parameters*/
static const char yaw_vel_name[] =   "Gimbal Yaw Vel";
static const char pitch_vel_name[] = "Gimbal Pitch Vel";
static const char axis_ff_name[] =   "Gimbal Axis FF";
static const char init_pos_name[] =  "Gimbal Init Pos";

static const char subname_axis[] = "Yaw Pitch";

/*
 *  @brief      Initialize the gimbal motor driver
 *  @NOTE       Requires to run can_processInit() first
 *
 *  @api
 */
void gimbal_init(void)
{
  memset(&gimbal, 0 ,sizeof(GimbalStruct));
  gimbal._encoder_can = can_getGimbalMotor();
  gimbal._pIMU = imu_get();
  chThdSleepMilliseconds(3);

  gimbal.pitch_speed = 0.0f;
  gimbal.yaw_speed = 0.0f;

  gimbal.yaw_iq_cmd = 0.0f;
  gimbal.pitch_iq_cmd = 0.0f;
  gimbal.yaw_iq_output = 0.0f;
  gimbal.pitch_iq_output = 0.0f;

  lpfilter_init(&lp_yawAngle, GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);
  lpfilter_init(&lp_pitchAngle, GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);

  gimbal_encoderUpdate();

  params_set(&(gimbal.yaw_vel),     0, 2, yaw_vel_name,   subname_PI,      PARAM_PUBLIC);
  params_set(&(gimbal.pitch_vel),   1, 2, pitch_vel_name, subname_PI,      PARAM_PUBLIC);
  params_set(gimbal.axis_ff_weight, 2, 2, axis_ff_name,   subname_axis,    PARAM_PUBLIC);

  params_set(gimbal.axis_init_pos,  5, 2, init_pos_name,  subname_axis,    PARAM_PUBLIC);

  chThdCreateStatic(gimbal_init_thread_wa, sizeof(gimbal_init_thread_wa),
                    NORMALPRIO - 5, gimbal_init_thread, NULL);

  gimbal.inited = true;
}
