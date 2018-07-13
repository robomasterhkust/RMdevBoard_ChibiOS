/**
 * Edward ZHANG, 20171101
 * @file    gimbal.c
 * @brief   Gimbal controller, driver and interface
 */
#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "attitude.h"
#include "math_misc.h"

#define GIMBAL_IQ_MAX 5000

static pi_controller_t _yaw_vel;
static pi_controller_t _pitch_vel;
static pid_controller_t _yaw_atti;
static pid_controller_t _pitch_atti;
static pid_controller_t _yaw_pos;
static pid_controller_t _pitch_pos;

#define gimbal_canUpdate()   \
  (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, \
    gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

static lpfilterStruct lp_yawAngle;
static lpfilterStruct lp_pitchAngle;
static GimbalStruct gimbal;

static thread_reference_t gimbal_thread_handler = NULL;

GimbalStruct* gimbal_get(void)
{
  return &gimbal;
}

uint32_t gimbal_getError(void)
{
  return gimbal.errorFlag;
}

static void gimbal_kill(void)
{
  gimbal.yaw_iq_output = 0;
  gimbal.pitch_iq_output = 0;
  gimbal_canUpdate();
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

static inline void gimbal_getSpeed(void)
{
  gimbal.pitch_speed = gimbal._pIMU->gyroData[Y];
  gimbal.yaw_speed = gimbal._pIMU->gyroData[Z] * cosf(gimbal.axis_ff_weight[5]) -
    gimbal._pIMU->gyroData[X] * sinf(gimbal.axis_ff_weight[5]);
}

static inline float gimbal_controlSpeed (pi_controller_t *const vel,
                                        const float target_vel,
                                        const float curr_vel)
{
  float error = target_vel - curr_vel;
  vel->error_int += error * vel->ki;
  bound(&(vel->error_int), vel->error_int_max);

  float output = vel->kp * error + vel->error_int;
  return output;
}

static inline float gimbal_controlAttitude (pid_controller_t *const atti,
                                            const float target_atti,
                                            const float curr_atti,
                                            const float curr_atti_d)
{
  float error = target_atti - curr_atti;
  atti->error_int += error * atti->ki;
  bound(&(atti->error_int), atti->error_int_max);

  //PI-D attitude controller
  float output = atti->kp * error + atti->error_int - atti->kd * curr_atti_d;
  return output;
}

#define GIMBAL_CONTROL_PERIOD_ST     US2ST(1000000U/GIMBAL_CONTROL_FREQ)
static THD_WORKING_AREA(gimbal_thread_wa, 2048);
static THD_FUNCTION(gimbal_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal controller");

//  chSysLock();
//  chThdSuspendS(&gimbal_thread_handler);
//  chSysUnlock();

  _yaw_vel.error_int = 0.0f;
  _pitch_vel.error_int = 0.0f;
  _yaw_atti.error_int = 0.0f;
  _pitch_atti.error_int = 0.0f;

  _yaw_vel.error_int_max = 2000.0f;
  _pitch_vel.error_int_max = 2500.0f;
  _yaw_atti.error_int_max = 4.0f;
  _pitch_atti.error_int_max = 4.0f;

  gimbal.yaw_speed_cmd = 0.0f;
  gimbal.pitch_speed_cmd = 0.0f;
  gimbal.yaw_atti_cmd = 0.0f;
  gimbal.pitch_atti_cmd = 0.436f;

  uint32_t tick = chVTGetSystemTimeX();
  float pitch_atti_out,yaw_atti_out;
  float sinroll, cosroll, cospitch;

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

    yaw_atti_out = gimbal_controlAttitude(&_yaw_atti,
                                      gimbal.yaw_atti_cmd,
                                      gimbal._pIMU->euler_angle[Yaw],
                                      gimbal._pIMU->d_euler_angle[Yaw]);
    pitch_atti_out = gimbal_controlAttitude(&_pitch_atti,
                                      gimbal.pitch_atti_cmd,
                                      gimbal._pIMU->euler_angle[Pitch],
                                      gimbal._pIMU->d_euler_angle[Pitch]);
    /*
     *  Jacobian output mixer
     *  wy = cos(roll)*pitch' + cos(pitch)*sin(roll)*yaw'
     *  wz = -sin(roll)*pitch' + cos(roll)*cos(pitch)*yaw'
     *  wyaw = wz / cos(theta)
     */
    float yaw_theta1 = gimbal.pitch_angle - gimbal.axis_ff_weight[3];
    float yaw_theta3 = yaw_theta1 + gimbal.axis_ff_weight[5];
    float yaw_theta4 = gimbal._pIMU->euler_angle[Pitch] - yaw_theta1;

    sinroll = sinf(gimbal._pIMU->euler_angle[Roll]);
    cosroll = cosf(gimbal._pIMU->euler_angle[Roll]);
    cospitch = cosf(gimbal._pIMU->euler_angle[Pitch]);

    //TODO Yaw mixer is not aligned
    gimbal.yaw_speed_cmd = -sinroll * pitch_atti_out + cospitch * cosroll * yaw_atti_out;
    gimbal.yaw_speed_cmd /= cosf(yaw_theta1);
    gimbal.pitch_speed_cmd = cosroll * pitch_atti_out + cospitch * sinroll * yaw_atti_out;

    gimbal.yaw_iq_cmd = gimbal_controlSpeed(&_yaw_vel,
      gimbal.yaw_speed_cmd, gimbal.yaw_speed);
    gimbal.pitch_iq_cmd = gimbal_controlSpeed(&_pitch_vel,
      gimbal.pitch_speed_cmd, gimbal.pitch_speed);

    /*
     *  @brief      Utilize the gimbal kinamatics model and add a feed-forward output
     *  @NOTE       Requires accelerometer raw data
     */
    float ff_pitch = norm_vector3_projection(gimbal._pIMU->accelData, gimbal.axis_ff_accel);
    gimbal.pitch_iq_cmd += gimbal.axis_ff_weight[GIMBAL_PITCH] * ff_pitch;

    /*
     * the reference acceleration vector is calculated in this way
     * RefX = G * cos(tan-1(M2*cos(theta)/M1)) * cos(theta + pitch0)
     * RefY = G * sin(tan-1(M2*cos(theta)/M1))
     * RefZ = G * cos(tan-1(M2*cos(theta)/M1)) * sin(theta + pitch0)
     * useful simplification : cos(tan-1(x)) = 1/sqrt(1+x^2)
     *                         sin(tan-1(x)) = x/sqrt(1+x^2)
     */
    float yaw_theta2 = gimbal.axis_ff_weight[4]*cosf(yaw_theta1) / gimbal.axis_ff_weight[GIMBAL_YAW];
    float temp = sqrtf(1 + yaw_theta2 * yaw_theta2);

    float a_ref[3];
    a_ref[X] = -(GRAV / temp) * cosf(yaw_theta3);
    a_ref[Y] = -(GRAV * yaw_theta2) / temp;
    a_ref[Z] = -(GRAV / temp) * sinf(yaw_theta3);
    float ff_yaw = norm_vector3_projection(gimbal._pIMU->accelData, a_ref);
    gimbal.yaw_iq_cmd +=
      (gimbal.axis_ff_weight[GIMBAL_YAW] + gimbal.axis_ff_weight[4] * cosf(yaw_theta1)) * ff_yaw;

    //Scale down gimbal yaw power according to pitch angle
    gimbal.yaw_iq_cmd -= cosf(yaw_theta1) * gimbal.axis_init_pos[2] * gimbal.yaw_iq_cmd;

    /*output limit*/
    gimbal.yaw_iq_output = boundOutput(gimbal.yaw_iq_cmd, GIMBAL_IQ_MAX);
    gimbal.pitch_iq_output = boundOutput(gimbal.pitch_iq_cmd, GIMBAL_IQ_MAX);

    gimbal_canUpdate();

    //Stop the thread while calibrating IMU
    /*
    if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    {
      gimbal_kill();
      chThdExit(MSG_OK);
    }*/
  }
}

#define GIMBAL_INIT_MAX_ERROR    8.7266e-4
#define GIMBAL_INIT_SCORE_FULL       1000U
#define GIMBAL_INIT_TIMEOUT         10000U
#define GIMBAL_POS_BUFFER_LEN 40U
static THD_WORKING_AREA(gimbal_init_thread_wa, 2048);
static THD_FUNCTION(gimbal_init_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal init thread");

  float _error[2];
  float _error_diff[2];
  const float _error_int_max[2] = {1000.0f, 1500.0f};
  float _error_past[GIMBAL_POS_BUFFER_LEN][2];
  uint8_t i;
  for (i = 0; i < GIMBAL_POS_BUFFER_LEN; i++)
  {
    _error_past[i][0] = 0.0f;
    _error_past[i][1] = 0.0f;
  }
  uint8_t _error_index = 0, _prev_index = 1;

  _pitch_pos.error_int = 0.0f;
  _yaw_pos.error_int = 0.0f;

  _pitch_pos.error_int_max = 1500.0f;
  _yaw_pos.error_int_max = 1000.0f;

  uint16_t _init_score[2] = {0, 0};
  uint16_t _init_time = 0;
  uint16_t _wait_time = 0;

  while(true)
  {
    gimbal_encoderUpdate();

    _error[GIMBAL_YAW] = gimbal.axis_init_pos[GIMBAL_YAW] - gimbal.yaw_angle;
    _error[GIMBAL_PITCH] = gimbal.axis_init_pos[GIMBAL_PITCH] - gimbal.pitch_angle;

    _yaw_pos.error_int += _error[GIMBAL_YAW] * _yaw_pos.ki;
    _pitch_pos.error_int += _error[GIMBAL_PITCH] * _pitch_pos.ki;

    _prev_index = (_error_index + 1) % GIMBAL_POS_BUFFER_LEN;

    _error_past[_error_index][GIMBAL_PITCH] = _error[GIMBAL_PITCH];
    _error_past[_error_index][GIMBAL_YAW] = _error[GIMBAL_YAW];
    if(_init_time > GIMBAL_POS_BUFFER_LEN)
    {
      _error_diff[GIMBAL_PITCH] = _error[GIMBAL_PITCH] - _error_past[_prev_index][GIMBAL_PITCH];
      _error_diff[GIMBAL_YAW]   = _error[GIMBAL_YAW]   - _error_past[_prev_index][GIMBAL_YAW];
    }
    else
    {
      _error_diff[GIMBAL_PITCH] = 0.0f;
      _error_diff[GIMBAL_YAW]   = 0.0f;
    }
    _error_index = _prev_index;

    bound(&_pitch_pos.error_int, _pitch_pos.error_int_max);
    bound(&_yaw_pos.error_int, _yaw_pos.error_int_max);

    gimbal.yaw_iq_output =
      _error[GIMBAL_YAW]*_yaw_pos.kp  + _yaw_pos.error_int + _error_diff[GIMBAL_YAW]*_yaw_pos.kd;
    gimbal.pitch_iq_output =
      _error[GIMBAL_PITCH]*_pitch_pos.kp + _pitch_pos.error_int + _error_diff[GIMBAL_PITCH]*_pitch_pos.kd;

    bound(&gimbal.yaw_iq_output, GIMBAL_IQ_MAX);
    bound(&gimbal.pitch_iq_output, GIMBAL_IQ_MAX);

    gimbal_canUpdate();

    //evaluation of init status
    for (i = 0; i < 2; i++)
    {
      if(_error[i] < GIMBAL_INIT_MAX_ERROR && _error[i] > -GIMBAL_INIT_MAX_ERROR)
        _init_score[i]++;
      else if(_init_score[i] > 100U)
        _init_score[i] -= 100;
      else
        _init_score[i] = 0;
    }

    chThdSleepMilliseconds(1);
  }
}

/* name of gimbal parameters*/
static const char axis_ff_name[]  = "Gimbal Axis FF";
static const char init_pos_name[] = "Gimbal Init Pos";
static const char accl_name[]     = "Gimbal FF Accl";
static const char subname_axis[]  = "Yaw Pitch Yaw_SD";
static const char subname_ff[]    = "Yaw_w1 Pitch_w Yaw_a Pitch_a Yaw_w2 Yaw_th";
static const char subname_accl[]  = "YawX YawY YawZ PitchX PitchY PitchZ";
static const char _yaw_vel_name[] =   "Gimbal Yaw Vel";
static const char _pitch_vel_name[] = "Gimbal Pitch Vel";
static const char _yaw_atti_name[] =   "Gimbal Yaw Atti";
static const char _pitch_atti_name[] = "Gimbal Pitch Atti";
static const char yaw_pos_name[] =   "Gimbal Yaw Pos";
static const char pitch_pos_name[] = "Gimbal Pitch Pos";

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

  params_set(gimbal.axis_init_pos,  5, 3, init_pos_name,  subname_axis,    PARAM_PUBLIC);
  params_set(gimbal.axis_ff_weight, 2, 6, axis_ff_name,   subname_ff,      PARAM_PUBLIC);
  params_set(gimbal.axis_ff_accel,  6, 6, accl_name,      subname_accl,    PARAM_PUBLIC);

  params_set(&_yaw_pos,   3, 3, yaw_pos_name,   subname_PID,   PARAM_PUBLIC);
  params_set(&_pitch_pos, 4, 3, pitch_pos_name, subname_PID,   PARAM_PUBLIC);

  params_set(&_yaw_vel,     0, 2, _yaw_vel_name,   subname_PI,      PARAM_PUBLIC);
  params_set(&_pitch_vel,   1, 2, _pitch_vel_name, subname_PI,      PARAM_PUBLIC);

  params_set(&_yaw_atti,     7, 3, _yaw_atti_name,   subname_PID,      PARAM_PUBLIC);
  params_set(&_pitch_atti,   8, 3, _pitch_atti_name, subname_PID,      PARAM_PUBLIC);

//  chThdCreateStatic(gimbal_init_thread_wa, sizeof(gimbal_init_thread_wa),
//                    NORMALPRIO - 5, gimbal_init_thread, NULL);
//  chThdCreateStatic(gimbal_thread_wa, sizeof(gimbal_thread_wa),
//                    NORMALPRIO - 5, gimbal_thread, NULL);

  gimbal.inited = true;
}
