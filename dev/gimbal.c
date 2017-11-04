/**
 * Edward ZHANG, 20171101
 * @file    gimbal.c
 * @brief   Gimbal controller, driver and interface
 */
#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "math_misc.h"

#define gimbal_canUpdate()   \
  (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, \
    gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

static volatile lpfilterStruct lp_yawAngle;
static volatile lpfilterStruct lp_pitchAngle;
static volatile GimbalStruct gimbal;

volatile GimbalStruct* gimbal_get(void)
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

#define GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define GIMBAL_CONNECTION_ERROR_COUNT 20U
static void gimbal_encoderUpdate(void)
{
  if(gimbal.encoder_can[GIMBAL_YAW].updated)
  {
    //Check validiaty of can connection
    gimbal.encoder_can[GIMBAL_YAW].updated = false;

    float angle_input =
      gimbal.encoder_can[GIMBAL_YAW].raw_angle*GIMBAL_ANGLE_PSC + GIMBAL_YAW_ANGLE_OFFSET;
    gimbal.yaw_angle = lpfilter_apply(&lp_yawAngle, angle_input);
    gimbal.yaw_current = gimbal.encoder_can[GIMBAL_YAW].raw_current;

    /* add FIR filter to encoder speed*/
    #ifdef GIMBAL_ENCODER_USE_SPEED
      yaw_count_sum += gimbal.yaw_wait_count;
      yaw_count_sum -= yaw_speed_count_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      int16_t diff = gimbal.encoder_can[GIMBAL_YAW].raw_angle -
        yaw_speed_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      //Detect zero-crossing scenerio
      if(diff > 6000)
        diff -= 8192;
      else if(diff < -6000)
        diff += 8192;

      gimbal.yaw_speed_enc = diff*GIMBAL_ANGLE_PSC * GIMBAL_CONTROL_FREQ/yaw_count_sum;
      yaw_speed_buffer[yaw_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal.encoder_can[GIMBAL_YAW].raw_angle;
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

  if(gimbal.encoder_can[GIMBAL_PITCH].updated)
  {
    //Check validiaty of can connection
    gimbal.encoder_can[GIMBAL_PITCH].updated = false;

    float angle_input =
      gimbal.encoder_can[GIMBAL_PITCH].raw_angle*GIMBAL_ANGLE_PSC + GIMBAL_PITCH_ANGLE_OFFSET;
    gimbal.pitch_angle = lpfilter_apply(&lp_pitchAngle, angle_input);
    gimbal.pitch_current = gimbal.encoder_can[GIMBAL_PITCH].raw_current;

    /* add FIR filter to encoder speed*/
    #ifdef GIMBAL_ENCODER_USE_SPEED
      pitch_count_sum += gimbal.pitch_wait_count;
      pitch_count_sum -= pitch_speed_count_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN];

      int16_t diff = gimbal.encoder_can[GIMBAL_PITCH].raw_angle -
        pitch_speed_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN];
      //Detect zero-crossing scenerio
      if(diff > 4096)
        diff -= 8192;
      else if(diff < -4096)
        diff += 8192;

      gimbal.pitch_speed_enc = diff*GIMBAL_ANGLE_PSC * GIMBAL_CONTROL_FREQ/pitch_count_sum;
      pitch_speed_buffer[pitch_speed_count % GIMBAL_SPEED_BUFFER_LEN] = gimbal.encoder_can[GIMBAL_PITCH].raw_angle;
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

#define GIMBAL_CONTROL_PERIOD_ST     US2ST(1000000U/GIMBAL_CONTROL_FREQ)
static THD_WORKING_AREA(gimbal_thread_wa, 2048);
static THD_FUNCTION(gimbal_thread, p)
{
  (void)p;
  chRegSetThreadName("Gimbal controller");

  param_t* params = params_get();
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

    if(gimbal.yaw_vel->kp < 1000.0f && gimbal.yaw_vel->kp > -1000.0f)
        gimbal.yaw_iq_output = gimbal.yaw_vel->kp;

    if(gimbal.yaw_vel->ki < 1000.0f && gimbal.yaw_vel->ki > -1000.0f)
        gimbal.pitch_iq_output = gimbal.yaw_vel->ki;

    gimbal_canUpdate();
  }
}

/*
 *  @brief      Initialize the gimbal motor driver
 *  @NOTE       Requires to run can_processInit() first
 *
 *  @api
 */
void gimbal_init(void)
{
  memset(&gimbal, 0 ,sizeof(GimbalStruct));
  gimbal.encoder_can = can_getGimbalMotor();
  chThdSleepMilliseconds(3);

  gimbal.yaw_iq_cmd = 0.0f;
  gimbal.pitch_iq_cmd = 0.0f;
  gimbal.yaw_iq_output = 0.0f;
  gimbal.pitch_iq_output = 0.0f;

  lpfilter_init(&lp_yawAngle, GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);
  lpfilter_init(&lp_pitchAngle, GIMBAL_CONTROL_FREQ, GIMBAL_CUTOFF_FREQ);

  gimbal_encoderUpdate();

  gimbal.yaw_vel = (gimbal_vel_controller_t*)(params_set(0,2));
  gimbal.pitch_vel = (gimbal_vel_controller_t*)(params_set(2,2));

  chThdCreateStatic(gimbal_thread_wa, sizeof(gimbal_thread_wa),
                    NORMALPRIO - 5, gimbal_thread, NULL);

  gimbal.inited = true;
}
