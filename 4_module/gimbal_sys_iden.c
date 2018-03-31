/**
 * Created by beck on 6/12/2017.
 * @brief   a simply cascaded gimbal controller
 *          and a gimbal system identification system
 */

#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "gimbal.h"
#include "dbus.h"
#include "string.h"
#include "math_misc.h"
#include "arm_math.h"

#define GIMBAL_SYSTEM_IDENTIFICATION false

GimbalStruct gimbal;
static arm_pid_instance_f32 pid_yaw_pos;
static arm_pid_instance_f32 pid_yaw_vel;
static arm_pid_instance_f32 pid_pitch_pos;
static arm_pid_instance_f32 pid_pitch_vel;

static pi_controller_t _yaw_vel;
static pi_controller_t _pitch_vel;
static pid_controller_t _yaw_atti;
static pid_controller_t _pitch_atti;

static THD_WORKING_AREA(gimbal_rc_control_wa, 512);
static THD_WORKING_AREA(gimbal_serial_control_wa, 512);
static THD_WORKING_AREA(gimbal_simple_control_wa, 2048);
static THD_WORKING_AREA(gimbal_sys_iden_wa, 2048);

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

        gimbal.yaw_angle   = gimbal._encoder_can[GIMBAL_YAW].radian_angle;
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

        gimbal.pitch_angle = gimbal._encoder_can[GIMBAL_PITCH].radian_angle;
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

static inline void pid_initialize(arm_pid_instance_f32* pid, const float Kp, const float Ki, const float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    arm_pid_init_f32(pid, 0);
}

static THD_FUNCTION(gimbal_rc_control, p)
{
    (void)p;
    chRegSetThreadName("chassis controller");
    RC_Ctl_t* pRC = RC_get();

    while (!chThdShouldTerminateX())
    {
        gimbal.pitch_atti_cmd += map(pRC->rc.channel3, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, (float)M_PI/2000, (float)-M_PI/2000);
        gimbal.yaw_atti_cmd   += map(pRC->rc.channel2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, (float)M_PI/700,  (float)-M_PI/700);
        bound(&gimbal.pitch_atti_cmd, (float)M_PI/8);
        bound(&gimbal.yaw_atti_cmd, (float)M_PI); // change to M_PI/3 if gear ratio changed
        chThdSleepMilliseconds(1);
    }
}

/**
 * @brief   Controller designed by Beck
 *          A simple cascaded PID controller
 * @ref     frame transformation from Pixhawk
 */
static THD_FUNCTION(gimbal_simple_control, p)
{
    (void)p;
    chRegSetThreadName("gimbal simple cascaded controller");
    gimbal.pitch_speed_cmd = 0.0f;
    gimbal.yaw_speed_cmd = 0.0f;

    uint32_t tick = chVTGetSystemTimeX();
    gimbal.timestamp = tick;

    while (!chThdShouldTerminateX())
    {
        gimbal_encoderUpdate();

        pid_initialize(&pid_pitch_pos, _pitch_atti.kp, _pitch_atti.ki, _pitch_atti.kd);
        pid_initialize(&pid_pitch_vel, _pitch_vel.kp, _pitch_vel.ki, 0); // -1000, 0, 0
        pid_initialize(&pid_yaw_pos, _yaw_atti.kp, _yaw_atti.ki, _yaw_atti.kd);
        pid_initialize(&pid_yaw_vel, _yaw_vel.kp, _yaw_vel.ki, 0); // 1000, 0, 0

        gimbal.pitch_speed_cmd = arm_pid_f32(&pid_pitch_pos, gimbal.pitch_atti_cmd + gimbal.axis_init_pos[GIMBAL_PITCH] - gimbal.pitch_angle);
        gimbal.yaw_speed_cmd = arm_pid_f32(&pid_yaw_pos, gimbal.yaw_atti_cmd + gimbal.axis_init_pos[GIMBAL_YAW] - gimbal.yaw_angle);

        gimbal.pitch_iq_cmd = arm_pid_f32(&pid_pitch_vel, gimbal.pitch_speed_cmd - gimbal._pIMU->gyroData[Y]);
        gimbal.yaw_iq_cmd   = arm_pid_f32(&pid_yaw_vel,   gimbal.yaw_speed_cmd - gimbal._pIMU->gyroData[Z]);

        gimbal.yaw_iq_output = boundOutput(gimbal.yaw_iq_cmd, GIMBAL_IQ_MAX);
        gimbal.pitch_iq_output = boundOutput(gimbal.pitch_iq_cmd, GIMBAL_IQ_MAX);

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

    float stop_time_table[30] = {0};
    stop_time_table[0] = P * (1 / freq_log_space[0]);

    int i;
    for (i = 0; i < 30; ++i) {
        stop_time_table[i] = stop_time_table[i-1] + P * (1 / freq_log_space[i]);
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

/**
 * @Note    Requires to run can_processInit() and gimbal_simple_controller_init() first
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

    params_set(gimbal.axis_init_pos,  5, 3, init_pos_name,  subname_axis,    PARAM_PUBLIC);
    params_set(gimbal.axis_ff_weight, 2, 6, axis_ff_name,   subname_ff,      PARAM_PUBLIC);
    params_set(gimbal.axis_ff_accel,  6, 6, accl_name,      subname_accl,    PARAM_PUBLIC);

//    params_set(&_yaw_pos,   3, 3, yaw_pos_name,   subname_PID,   PARAM_PUBLIC);
//    params_set(&_pitch_pos, 4, 3, pitch_pos_name, subname_PID,   PARAM_PUBLIC);

    params_set(&_yaw_vel,     0, 2, _yaw_vel_name,   subname_PI,      PARAM_PUBLIC);
    params_set(&_pitch_vel,   1, 2, _pitch_vel_name, subname_PI,      PARAM_PUBLIC);

    params_set(&_yaw_atti,     7, 3, _yaw_atti_name,   subname_PID,      PARAM_PUBLIC);
    params_set(&_pitch_atti,   8, 3, _pitch_atti_name, subname_PID,      PARAM_PUBLIC);

    arm_pid_init_f32(&pid_pitch_pos, 1);
    arm_pid_init_f32(&pid_pitch_vel, 1);
    arm_pid_init_f32(&pid_yaw_pos, 1);
    arm_pid_init_f32(&pid_yaw_vel, 1);

    gimbal_encoderUpdate();

    chThdCreateStatic(gimbal_simple_control_wa, sizeof(gimbal_simple_control_wa),
                        NORMALPRIO - 5, gimbal_simple_control, NULL);

    chThdCreateStatic(gimbal_rc_control_wa, sizeof(gimbal_rc_control_wa),
                        NORMALPRIO - 5, gimbal_rc_control, NULL);

#if GIMBAL_SYSTEM_IDENTIFICATION
    chThdCreateStatic(gimbal_sys_iden_wa, sizeof(gimbal_sys_iden_wa),
                      NORMALPRIO - 5, gimbal_sys_iden, NULL);
#endif

    gimbal.inited = true;
}
