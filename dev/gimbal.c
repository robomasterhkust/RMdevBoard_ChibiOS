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

#define GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define GIMBAL_CONNECTION_ERROR_COUNT 10U
static void gimbal_encoderUpdate(void)
{
  if(gimbal.encoder_can[YAW].updated)
  {
    //Check validiaty of can connection
    gimbal.encoder_can[YAW].updated = false;
    gimbal.yaw_wait_count = 1;

    float angle_input =
      gimbal.encoder_can[YAW].raw_angle*GIMBAL_ANGLE_PSC + GIMBAL_YAW_ANGLE_OFFSET;
    gimbal.yaw_angle = lpfilter_apply(&lp_yawAngle, angle_input);
    gimbal.yaw_current = gimbal.encoder_can[YAW].raw_current;

    /* TODO: add FIR filter to encoder speed*/
    #ifdef GIMBAL_USE_SPEED
    #endif
  }
  else
  {
    gimbal.yaw_wait_count++;
    if(gimbal.yaw_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
    {
      gimbal.errorFlag |= GIMBAL_YAW_NOT_CONNECTED;
      gimbal.pitch_wait_count = 0;
    }
  }

  if(gimbal.encoder_can[PITCH].updated)
  {
    //Check validiaty of can connection
    gimbal.encoder_can[PITCH].updated = false;
    gimbal.pitch_wait_count = 1;

    float angle_input =
      gimbal.encoder_can[PITCH].raw_angle*GIMBAL_ANGLE_PSC + GIMBAL_PITCH_ANGLE_OFFSET;
    gimbal.pitch_angle = lpfilter_apply(&lp_pitchAngle, angle_input);
    gimbal.pitch_current = gimbal.encoder_can[PITCH].raw_current;

    /* TODO: add FIR filter to encoder speed*/
    #ifdef GIMBAL_USE_SPEED
    #endif
  }
  else
  {
    gimbal.pitch_wait_count++;
    if(gimbal.pitch_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
    {
      gimbal.errorFlag |= GIMBAL_PITCH_NOT_CONNECTED;
      gimbal.pitch_wait_count = 0;
    }
  }
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

  chThdCreateStatic(gimbal_thread_wa, sizeof(gimbal_thread_wa),
                    NORMALPRIO - 5, gimbal_thread, NULL);

  gimbal.inited = true;
}
