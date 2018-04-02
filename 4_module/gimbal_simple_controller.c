/**
 * Beck Pang, 20180329, a simple cascaded gimbal controller
 *          Using only two cascaded PID controller
 *          with no model or the feedforward term
 *          Used for initial installation and system identification
 * @input: RM6623 encoder, onboard IMU
 */
#include "gimbal.h"
#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "arm_math.h"
#include "command_mixer_task.h"

GimbalStruct gimbal;
static arm_pid_instance_f32 pid_yaw_pos;
static arm_pid_instance_f32 pid_yaw_vel;
static arm_pid_instance_f32 pid_pitch_pos;
static arm_pid_instance_f32 pid_pitch_vel;

static pi_controller_t _yaw_vel;
static pi_controller_t _pitch_vel;
static pid_controller_t _yaw_atti;
static pid_controller_t _pitch_atti;

static THD_WORKING_AREA(gimbal_simple_controller_wa, 2048);

/* name of gimbal parameters */
static const char axis_ff_name[] = "Gimbal Axis FF";
static const char init_pos_name[] = "Gimbal Init Pos";
static const char accl_name[] = "Gimbal FF Accl";
static const char subname_axis[] = "Yaw Pitch Yaw_SD";
static const char subname_ff[] = "Yaw_w1 Pitch_w Yaw_a Pitch_a Yaw_w2 Yaw_th";
static const char subname_accl[] = "YawX YawY YawZ PitchX PitchY PitchZ";
static const char _yaw_vel_name[] = "Gimbal Yaw Vel";
static const char _pitch_vel_name[] = "Gimbal Pitch Vel";
static const char _yaw_atti_name[] = "Gimbal Yaw Atti";
static const char _pitch_atti_name[] = "Gimbal Pitch Atti";
static const char yaw_pos_name[] = "Gimbal Yaw Pos";
static const char pitch_pos_name[] = "Gimbal Pitch Pos";

#define gimbal_canUpdate() \
    (can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,\
        gimbal.yaw_iq_output, gimbal.pitch_iq_output, 0, 0))

GimbalStruct *get_gimbal_simple_controller(void)
{
    return &gimbal;
}

static void gimbal_encoderUpdate(void)
{
    if (gimbal._encoder_can[GIMBAL_YAW].updated) {
        gimbal._encoder_can[GIMBAL_YAW].updated = false;

        gimbal.yaw_angle = gimbal._encoder_can[GIMBAL_YAW].radian_angle;
        gimbal.yaw_current = gimbal._encoder_can[GIMBAL_YAW].raw_current;

        gimbal.yaw_wait_count = 1;
    } else {
        gimbal.yaw_wait_count++;
        if (gimbal.yaw_wait_count > GIMBAL_CONNECTION_ERROR_COUNT) {
            gimbal.errorFlag |= GIMBAL_YAW_NOT_CONNECTED;
            gimbal.yaw_wait_count = 1;
        }
    }

    if (gimbal._encoder_can[GIMBAL_PITCH].updated) {
        gimbal._encoder_can[GIMBAL_PITCH].updated = false;

        gimbal.pitch_angle = gimbal._encoder_can[GIMBAL_PITCH].radian_angle;
        gimbal.pitch_current = gimbal._encoder_can[GIMBAL_PITCH].raw_current;

        gimbal.pitch_wait_count = 1;
    } else {
        gimbal.pitch_wait_count++;
        if (gimbal.pitch_wait_count > GIMBAL_CONNECTION_ERROR_COUNT) {
            gimbal.errorFlag |= GIMBAL_PITCH_NOT_CONNECTED;
            gimbal.pitch_wait_count = 1;
        }
    }
}

static inline void pid_initialize(arm_pid_instance_f32 *pid, const float Kp, const float Ki, const float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    arm_pid_init_f32(pid, 0);
}

/**
 * @brief   Controller designed by Beck
 *          A simple cascaded PID controller
 * @ref     frame transformation from Pixhawk
 * @input   Remote Controller
 */
static THD_FUNCTION(gimbal_simple_controller, p)
{
    (void) p;
    chRegSetThreadName("gimbal simple cascaded controller");
    gimbal.pitch_speed_cmd = 0.0f;
    gimbal.yaw_speed_cmd = 0.0f;

    uint32_t tick = chVTGetSystemTimeX();
    gimbal.timestamp = tick;

    GimbalCommandStruct* gimbal_cmd = gimbal_cmd_get();

    while (!chThdShouldTerminateX()) {
        gimbal_encoderUpdate();

        pid_initialize(&pid_pitch_pos, _pitch_atti.kp, _pitch_atti.ki, _pitch_atti.kd);
        pid_initialize(&pid_pitch_vel, _pitch_vel.kp, _pitch_vel.ki, 0); // -1000, 0, 0
        pid_initialize(&pid_yaw_pos, _yaw_atti.kp, _yaw_atti.ki, _yaw_atti.kd);
        pid_initialize(&pid_yaw_vel, _yaw_vel.kp, _yaw_vel.ki, 0); // 1000, 0, 0

        gimbal.pitch_speed_cmd = arm_pid_f32(&pid_pitch_pos,
                                             gimbal_cmd->pitch_atti_cmd + gimbal.axis_init_pos[GIMBAL_PITCH] -
                                             gimbal.pitch_angle);
        gimbal.yaw_speed_cmd = arm_pid_f32(&pid_yaw_pos,
                                           gimbal_cmd->yaw_atti_cmd + gimbal.axis_init_pos[GIMBAL_YAW] - gimbal.yaw_angle);

        gimbal.pitch_iq_cmd = arm_pid_f32(&pid_pitch_vel, gimbal.pitch_speed_cmd - gimbal._pIMU->gyroData[Y]);
        gimbal.yaw_iq_cmd = arm_pid_f32(&pid_yaw_vel, gimbal.yaw_speed_cmd - gimbal._pIMU->gyroData[Z]);

        gimbal.yaw_iq_output = boundOutput(gimbal.yaw_iq_cmd, GIMBAL_IQ_MAX);
        gimbal.pitch_iq_output = boundOutput(gimbal.pitch_iq_cmd, GIMBAL_IQ_MAX);

        gimbal.timestamp = chVTGetSystemTimeX();

        tick += GIMBAL_CONTROL_PERIOD_NEXT;
        if (tick > gimbal.timestamp) {
            chThdSleepUntil(tick);
        } else {
            tick = gimbal.timestamp;
            gimbal.errorFlag |= GIMBAL_CONTROL_LOSE_FRAME;
        }
        gimbal_canUpdate();
    }
}

/**
 * @Note    Requires to run can_processInit() and gimbal_simple_controller_init() first
 */
void gimbal_simpler_controller_init(void)
{
    memset(&gimbal, 0, sizeof(GimbalStruct));
    gimbal._encoder_can = can_getGimbalMotor();
    gimbal._pIMU = imu_get();
    chThdSleepMilliseconds(3);

    gimbal.pitch_speed = 0.0f;
    gimbal.yaw_speed = 0.0f;

    gimbal.yaw_iq_cmd = 0.0f;
    gimbal.pitch_iq_cmd = 0.0f;
    gimbal.yaw_iq_output = 0.0f;
    gimbal.pitch_iq_output = 0.0f;

    params_set(gimbal.axis_init_pos, 5, 3, init_pos_name, subname_axis, PARAM_PUBLIC);
//    params_set(gimbal.axis_ff_weight, 2, 6, axis_ff_name,   subname_ff,      PARAM_PUBLIC);
//    params_set(gimbal.axis_ff_accel,  6, 6, accl_name,      subname_accl,    PARAM_PUBLIC);

//    params_set(&_yaw_pos,   3, 3, yaw_pos_name,   subname_PID,   PARAM_PUBLIC);
//    params_set(&_pitch_pos, 4, 3, pitch_pos_name, subname_PID,   PARAM_PUBLIC);

    params_set((param_t*)&_yaw_vel, 0, 2, _yaw_vel_name, subname_PI, PARAM_PUBLIC);
    params_set((param_t*)&_pitch_vel, 1, 2, _pitch_vel_name, subname_PI, PARAM_PUBLIC);

    params_set((param_t*)&_yaw_atti, 7, 3, _yaw_atti_name, subname_PID, PARAM_PUBLIC);
    params_set((param_t*)&_pitch_atti, 8, 3, _pitch_atti_name, subname_PID, PARAM_PUBLIC);

    arm_pid_init_f32(&pid_pitch_pos, 1);
    arm_pid_init_f32(&pid_pitch_vel, 1);
    arm_pid_init_f32(&pid_yaw_pos, 1);
    arm_pid_init_f32(&pid_yaw_vel, 1);

    gimbal_encoderUpdate();

    chThdCreateStatic(gimbal_simple_controller_wa, sizeof(gimbal_simple_controller_wa),
                      NORMALPRIO - 5, gimbal_simple_controller, NULL);

    gimbal.inited = true;

}