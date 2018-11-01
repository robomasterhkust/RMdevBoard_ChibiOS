/**
 * chassis core library
 *      - chassis thread
 *      - mechanum wheel kinematics
 *      - chassis and power control policy
 *      - state machine
 * moved communication to can_comm
 * simplified the power control policy
 *
 * 20181029 Beck Pang
 */
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "judge.h"
#include "chassis_task.h"
#include "command_mixer.h"
#include "math_misc.h"

static chassis_t chassis;

/*
 * public functions
 */
volatile chassis_t *chassis_struct_get(void){ return &chassis; }

chassis_error_t chassis_getError(void) { return chassis.error_flag; }

/*
 * private functions
 */
static void mecanum_inverse_kinematics
        (float vx, float vy, float w_radian);

static void clear_motor_speed_sp();

static void chassis_encoder_update
        (volatile ChassisEncoder_canStruct *encoderPtr);

static void power_limit_control(judge_fb_t* judgePtr);

static void velocity_control();

static void chassis_drive_motor();

static void chassis_state_machine(command_t* cmd, volatile GimbalEncoder_canStruct *encoder);

static void follow_gimbal_handler(float vx, float vy, float angle, pid_controller_t* heading_controller);

/*
 * private parameters
 */
bool                rotation_center_gimbal = false;
float               motor_speed_cutoff_freq = 10.0f;
lpfilterStruct      lp_motor_speed[CHASSIS_MOTOR_NUM];

/**
 * 1. Chassis thread
 */
static THD_WORKING_AREA(chassis_control_wa, 2048);

static THD_FUNCTION(chassis_control, p)
{
    (void) p;
    chRegSetThreadName("chassis controller");

    volatile ChassisEncoder_canStruct* encoderPtr = can_getChassisMotor();

    command_t* cmd_mixer = cmd_mixer_get();

    uint32_t tick = chVTGetSystemTimeX();

    chassis.joint_angle_ref = encoderPtr[GIMBAL_YAW].radian_angle;

    while (!chThdShouldTerminateX()) {
        tick += US2ST(CHASSIS_TASK_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX()) {
            chThdSleepUntil(tick);
        } else {
            tick = chVTGetSystemTimeX();
        }
        chassis_encoder_update(encoderPtr);

        chassis_state_machine(cmd_mixer, can_getGimbalMotor());

        // power_limit_control(judgeDataGet());

        velocity_control();
    }
}


void
chassis_task_init()
{
    memset(&chassis, 0, sizeof(chassis_t));
    chassis.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis.rotate_y_offset = GIMBAL_Y_OFFSET;

    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i) {
        memset(&chassis._motors, 0, sizeof(chassis_motor_t));
        memset(&chassis.motor_vel_ctrl[i], 0, sizeof(pi_controller_t));
        lpfilter_init(lp_motor_speed + i, CHASSIS_TASK_UPDATE_FREQ, motor_speed_cutoff_freq);
        chassis.motor_vel_ctrl[i].kp = 300.0f;
        chassis.motor_vel_ctrl[i].ki = 0.0f;
        chassis.motor_vel_ctrl[i].kd = 3.0f;
        chassis.motor_vel_ctrl[i].error_int_max = 1000.0f;
        chassis.motor_vel_ctrl[i].output_max = MOTOR_OUTPUT_MAX;
    }
    memset(&chassis.heading_ctrl, 0, sizeof(pid_controller_t));
    chassis.heading_ctrl[0].kp = 0.0f;
    chassis.heading_ctrl[0].ki = 0.0f;
    chassis.heading_ctrl[0].kd = 0.0f;

    chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa), NORMALPRIO,
                      chassis_control, NULL);
}

/**
 2. Mechanum wheel kinematics

           ^ x
           |
           |
     y<----O

      |   width  |
    - [1]-------[0]
    |      | |
  length   |O|
    |      | |
    - [2]-------[3]

 config of Mecanum Wheel:
      [\\]-------[//]
           | |
           | |
           | |
      [//]-------[\\]
 */

/*
 * Inverse Kinematics of the four Mecanum wheel system
 * @param chassisPtr
 * @param vx, 3300 ~ -3300, mm/s
 * @param vy, 3300 ~ -3300, mm/s
 * @param w,  degree/s
 * @output w1, w2, w3, w4, rad/s
 */
static void
mecanum_inverse_kinematics(float vx, float vy, float w_radian)
{
    float rotate_ratio_fr;
    float rotate_ratio_fl;
    float rotate_ratio_br;
    float rotate_ratio_bl;
    float lx = WHEELBASE * 0.5f;
    float ly = WHEELTRACK * 0.5f;
    float dx = chassis.rotate_x_offset;
    float dy = chassis.rotate_y_offset;
    float r = MECANUM_WHEEL_RADIUS;
    float vw = w_radian; // radian/s

    if (!rotation_center_gimbal) {
        // normal kinematics
        rotate_ratio_fr = (lx + ly);
        rotate_ratio_fl = (lx + ly);
        rotate_ratio_bl = (lx + ly);
        rotate_ratio_br = (lx + ly);
    } else {
        rotate_ratio_fr = (lx + ly - dx + dy);
        rotate_ratio_fl = (lx + ly - dx - dy);
        rotate_ratio_bl = (lx + ly + dx - dy);
        rotate_ratio_br = (lx + ly + dx + dy);
    }

    VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

//    chassis._motors[FRONT_RIGHT].speed_sp = (-1 * vx + vy + vw * rotate_ratio_fr) / r;
//    chassis._motors[FRONT_LEFT].speed_sp  = (vx + vy + vw * rotate_ratio_fl) / r;
//    chassis._motors[BACK_LEFT].speed_sp   = (vx - vy + vw * rotate_ratio_bl) / r;
//    chassis._motors[BACK_RIGHT].speed_sp  = (-1 * vx - vy + vw * rotate_ratio_br) / r;
    chassis._motors[FRONT_RIGHT].speed_sp = (vx + vy - vw * rotate_ratio_fr) / r;
    chassis._motors[FRONT_LEFT].speed_sp  = (vx - vy + vw * rotate_ratio_fl) / r;
    chassis._motors[BACK_LEFT].speed_sp   = (vx + vy + vw * rotate_ratio_bl) / r;
    chassis._motors[BACK_RIGHT].speed_sp  = (vx - vy - vw * rotate_ratio_br) / r;
}

static void
clear_motor_speed_sp() {
    chassis._motors[FRONT_RIGHT].speed_sp = 0;
    chassis._motors[FRONT_LEFT].speed_sp  = 0;
    chassis._motors[BACK_LEFT].speed_sp   = 0;
    chassis._motors[BACK_RIGHT].speed_sp  = 0;
}

/*
 * Update the motor speed to encoder speed in RPM
 * Change the motor direction to fit the defined orientation, flipped the front right and back right
 */
static void
chassis_encoder_update(volatile ChassisEncoder_canStruct *encoderPtr)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; ++i) {
        if (encoderPtr[i].updated) {
            encoderPtr[i].updated = false;

            if (i == 0 || i == 3)
                chassis._motors[i]._speed_raw = -encoderPtr[i].raw_speed * RM3508_MOTOR_GEAR_RATIO * RPM_TO_RAD_PER_SEC; // rad/s
            else
                chassis._motors[i]._speed_raw = encoderPtr[i].raw_speed * RM3508_MOTOR_GEAR_RATIO * RPM_TO_RAD_PER_SEC; // rad/s

            chassis._motors[i]._speed = lpfilter_apply(&lp_motor_speed[i], chassis._motors[i]._speed_raw);
            chassis._motors[i]._wait_count = 0;
        }
        else {
            chassis._motors[i]._wait_count++;
            if (chassis._motors[i]._wait_count > CHASSIS_CONNECTION_ERROR_COUNT)
            {
                chassis.error_flag |= CHASSIS_MOTOR_NOT_CONNECTED << i;
                chassis._motors[i]._wait_count = 0;
            }
        }
    }
}


/**
 * 3. Chassis control policy
 */

static void
chassis_drive_motor()
{
    can_motorSetCurrent(CHASSIS_CAN_PORT, CHASSIS_CAN_SID,
                        -chassis.current[FRONT_RIGHT],
                        chassis.current[FRONT_LEFT],
                        chassis.current[BACK_LEFT],
                        -chassis.current[BACK_RIGHT]);
}

/*
 * PI velocity control loop, send motor command
 */
static void
velocity_control()
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; ++i) {
        float error = chassis._motors[i].speed_sp - chassis._motors[i]._speed;
        chassis.current[i] = (int16_t) pid_control(error, &chassis.motor_vel_ctrl[i]);
    }
    chassis_drive_motor();
}

static void
power_limit_control(judge_fb_t* judgePtr)
{
    float energy_threshold = 40.0f; // Joules
    int32_t total_cur_threshold = 40000;

    float remain_energy = judgePtr->powerInfo.powerBuffer;
    int32_t total_cur_limit;

    // if (!judge_connected)
    // else
    if (remain_energy > energy_threshold) {
        total_cur_limit = total_cur_threshold;
    }
    else {
        // control
        total_cur_limit = (int32_t)(remain_energy * remain_energy / \
                            (energy_threshold * energy_threshold) * total_cur_threshold);
    }
    int32_t total_cur = abs(chassis.current[0]) + abs(chassis.current[1]) + \
                        abs(chassis.current[2]) + abs(chassis.current[3]);

    if (total_cur > total_cur_limit)
    {
        float ratio = (float)total_cur_limit / total_cur;
        chassis.current[0] = (int16_t)(chassis.current[0] * ratio);
        chassis.current[1] = (int16_t)(chassis.current[0] * ratio);
        chassis.current[2] = (int16_t)(chassis.current[0] * ratio);
        chassis.current[3] = (int16_t)(chassis.current[0] * ratio);
    }
}

/**
 * 4. State Machine
 */
static void
chassis_state_machine(command_t* cmd, volatile GimbalEncoder_canStruct *encoder){
    if (cmd->reboot_flag) {
        chassis.joint_angle_ref = encoder[GIMBAL_YAW].radian_angle;
    }

    switch (cmd->ctrl_mode) {
        case CTL_CHASSIS_RELAX:     clear_motor_speed_sp(); break;
        case CTL_CHASSIS_STOP:      clear_motor_speed_sp(); break;
        case CTL_MANUAL_SEPARATE_GIMBAL: break;
        case CTL_MANUAL_FOLLOW_GIMBAL:
            chassis.joint_angle_diff = (chassis.joint_angle_ref - encoder[GIMBAL_YAW].radian_angle) * GIMBAL_YAW_GEAR;
            follow_gimbal_handler(cmd->vx, cmd->vy, chassis.joint_angle_diff, chassis.heading_ctrl);
            break;
        case CTL_DODGE_MODE: break;
        case CTL_AUTO_SEPARATE_GIMBAL: break;
        case CTL_AUTO_FOLLOW_GIMBAL: break;
        case CTL_DODGE_MOVE_MODE: break;
        case CTL_SAVE_LIFE: break;
        default:break;
    }
}

// planar robot velocity command to follow an angle in radian
static void
follow_gimbal_handler(float vx, float vy, float angle, pid_controller_t* heading_controller)
{
    chassis.vx_sp = vx * cosf(angle) + vy * sinf(angle);
    chassis.vy_sp = vx * sinf(angle) - vy * cosf(angle);

    chassis.vw_sp = pid_control(angle, heading_controller);

    mecanum_inverse_kinematics(chassis.vx_sp, chassis.vy_sp, chassis.vw_sp);
}

