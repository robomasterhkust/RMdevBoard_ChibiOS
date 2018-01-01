//
// Created by beck on 6/12/2017.
//

#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "gimbal.h"
#include "string.h"
#include "math_misc.h"
#include "arm_math.h"

GimbalStruct gimbal;
static arm_pid_instance_f32 pid_yaw_pos;
 arm_pid_instance_f32 pid_yaw_vel;
static arm_pid_instance_f32 pid_pitch_pos;
 arm_pid_instance_f32 pid_pitch_vel;
#define GIMBAL_MAX_ANGLE 5.02655f
#define GIMBAL_MIN_ANGLE 1.25664f
#define GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define GIMBAL_CONNECTION_ERROR_COUNT 20U
#define GIMBAL_CONTROL_PERIOD_NEXT    US2ST(1000000U/GIMBAL_CONTROL_FREQ)

#define gimbal_canUpdate() \
    (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,\
        gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

GimbalStruct* gimbal_get_sys_iden(void)
{
    return &gimbal;
}

static void gimbal_encoderUpdate(void)
{
    if (gimbal._encoder_can[GIMBAL_YAW].updated)
    {
        gimbal._encoder_can[GIMBAL_YAW].updated = false;

        float angle_input = (float) (gimbal._encoder_can[GIMBAL_YAW].raw_angle * GIMBAL_ANGLE_PSC);

        // Prevent zero-crossing
        if (angle_input > GIMBAL_MAX_ANGLE && gimbal.yaw_angle < GIMBAL_MIN_ANGLE)
            angle_input -= 2*M_PI;
        else if (angle_input < GIMBAL_MIN_ANGLE && gimbal.yaw_angle > GIMBAL_MAX_ANGLE)
            angle_input += 2*M_PI;

        gimbal.yaw_angle   = angle_input;
        gimbal.yaw_current = gimbal._encoder_can[GIMBAL_YAW].raw_current;

        gimbal.yaw_wait_count = 1;
    }
    else
    {
        gimbal.yaw_wait_count++;
        if (gimbal.yaw_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
        {
            gimbal.errorFlag |= GIMBAL_YAW_NOT_CONNECTED;
            gimbal.yaw_wait_count = 1;
        }
    }

    if (gimbal._encoder_can[GIMBAL_PITCH].updated)
    {
        gimbal._encoder_can[GIMBAL_PITCH].updated = false;

        float angle_input = (float) (gimbal._encoder_can[GIMBAL_PITCH].raw_angle * GIMBAL_ANGLE_PSC);

        // Prevent zero-crossing
        if (angle_input > GIMBAL_MAX_ANGLE && gimbal.pitch_angle < GIMBAL_MIN_ANGLE)
            angle_input -= 2*M_PI;
        else if (angle_input < GIMBAL_MIN_ANGLE && gimbal.pitch_angle > GIMBAL_MAX_ANGLE)
            angle_input += 2*M_PI;

        gimbal.pitch_angle = angle_input;
        gimbal.pitch_current = gimbal._encoder_can[GIMBAL_PITCH].raw_current;

        gimbal.pitch_wait_count = 1;
    }
    else
    {
        gimbal.pitch_wait_count++;
        if (gimbal.pitch_wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
        {
            gimbal.errorFlag |= GIMBAL_PITCH_NOT_CONNECTED;
            gimbal.pitch_wait_count = 1;
        }
    }
}

static THD_WORKING_AREA(gimbal_control_wa, 2048);
static THD_WORKING_AREA(gimbal_sys_iden_wa, 2048);

/**
 * @brief   Controller designed by Beck
 * @ref     feedforward controller from Edward
 *          frame transformation from Pixhawk
 */
static THD_FUNCTION(gimbal_control, p)
{
    (void)p;
    chRegSetThreadName("gimbal controller");
    gimbal.pitch_speed_cmd = 0.0f;
    gimbal.yaw_speed_cmd = 0.0f;

    uint32_t tick = chVTGetSystemTimeX();
    gimbal.timestamp = tick;

    while (!chThdShouldTerminateX())
    {
        gimbal_encoderUpdate();

        gimbal.pitch_iq_output = arm_pid_f32(&pid_pitch_vel, gimbal.pitch_speed_cmd - gimbal._pIMU->gyroData[Y]);
        gimbal.yaw_iq_output   = arm_pid_f32(&pid_yaw_vel,   gimbal.yaw_speed_cmd - gimbal._pIMU->gyroData[Z]);
        gimbal.timestamp = chVTGetSystemTimeX();

        tick += GIMBAL_CONTROL_PERIOD_NEXT;
        if (tick > gimbal.timestamp){
            chThdSleepUntil(tick);
        }
        else {
            tick = gimbal.timestamp;
            gimbal.errorFlag |= GIMBAL_CONTROL_LOSE_FRAME;
        }
        gimbal_canUpdate();
    }
}

volatile float gain = 500.0f;
volatile long long current_time_ms;
volatile float stop_time_table[31];
volatile float current_time;
volatile float test_time;
volatile double excitation;
volatile int n;
uint32_t clock_counter = 0;
/**
 * Create excitement signal with N frequency, P period in each frequency
 * N = 30 from the matlab generation
 */
static THD_FUNCTION(gimbal_sys_iden, p)
{
    (void)p;
    chRegSetThreadName("gimbal system identification");

//    float gain = 5.0f;
    int P = 5;
    int N = 30;

    // Hardcoded sweeping frequency from 1Hz to 500Hz in log space
    float freq_log_space[30] = {1.0f,	1.23899037094209f,	1.53509713928722f,	1.90197057403762f,	    \
    2.35652322704781f,	2.91970958721362f,	3.61749206450498f,	4.48203783488109f,	5.55320171961581f,  \
    6.88036345850305f,	8.52470407366710f,	10.5620262624044f,	13.0862488367565f,	16.2137363004934f,  \
    20.0886631533056f,	24.8896602120448f,	30.8380493387440f,	38.2080461893409f,	47.3394013211040f,	\
    58.6530624030112f,	72.6705795435965f,	90.0381483052973f,	111.556398767719f,	138.217303890180f,	\
    171.249908617510f,	212.176987801808f,	262.885244821938f,	325.712286997135f,	403.554387286977f,	500.0f};
//    float stop_time_table[30] = {0};
    stop_time_table[0] = 0;
    for (int i = 1; i < N + 1; ++i) {
        stop_time_table[i] = stop_time_table[i - 1] + P * (1 / freq_log_space[i - 1]) * i;
    }

    int wait_period = 5;
    chThdSleep(GIMBAL_CONTROL_PERIOD_NEXT * wait_period);

    long long int start_time_ms = (long long int) ST2MS(chVTGetSystemTimeX());
    n = 1; // the number of frequency

    uint32_t tick = chVTGetSystemTimeX();

    while (!chThdShouldTerminateX())
    {
        long long int current_time_ms_ns = ((long long int) ST2MS(chVTGetSystemTimeX()) - start_time_ms);
        if (current_time_ms > current_time_ms_ns) {
            clock_counter++;
        }
        current_time_ms = current_time_ms_ns;
        current_time =
                (current_time_ms + (clock_counter * (0xFFFFFFFF - CH_CFG_ST_FREQUENCY + 1UL) / CH_CFG_ST_FREQUENCY)) /
                1000.0f;
        if (n < N) {
            if (current_time > stop_time_table[n]) {
                n++;
            }
        }

        float period_time = current_time - stop_time_table[n - 1];
        float frequency = freq_log_space[n];
        test_time = (float) (2 * M_PI * frequency * period_time);
        excitation = gain * sinf((float) (2 * M_PI * frequency * period_time));
        gimbal.yaw_iq_output += excitation;

        tick += GIMBAL_CONTROL_PERIOD_NEXT;
        if (tick > chVTGetSystemTimeX()){
            chThdSleepUntil(tick);
        }
    }
}

static void pid_initialize(arm_pid_instance_f32* pid, const float Kp, const float Ki, const float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    arm_pid_init_f32(pid, 0);
}

/**
 * @Note    Requires to run can_processInit() first
 */
void gimbal_sys_iden_init(void)
{
    memset(&gimbal, 0, sizeof(GimbalStruct));
    gimbal._encoder_can = can_getGimbalMotor();
    gimbal._pIMU = imu_get();
    chThdSleepMilliseconds(3);

    gimbal.pitch_speed      = 0.0f;
    gimbal.yaw_speed        = 0.0f;

    gimbal.yaw_iq_cmd       = 0.0f;
    gimbal.pitch_iq_cmd     = 0.0f;
    gimbal.yaw_iq_output    = 0.0f;
    gimbal.pitch_iq_output  = 0.0f;

    pid_initialize(&pid_pitch_pos, 10, 0, 0);
    pid_initialize(&pid_pitch_vel, 4000, 0, 0);
    pid_initialize(&pid_yaw_pos, 10, 0, 0);
    pid_initialize(&pid_yaw_vel, 1000, 0, 0);

    gimbal_encoderUpdate();

    chThdCreateStatic(gimbal_control_wa, sizeof(gimbal_control_wa),
                        NORMALPRIO - 5, gimbal_control, NULL);

    chThdCreateStatic(gimbal_sys_iden_wa, sizeof(gimbal_sys_iden_wa),
                      NORMALPRIO - 5, gimbal_sys_iden, NULL);

    gimbal.inited = true;
}