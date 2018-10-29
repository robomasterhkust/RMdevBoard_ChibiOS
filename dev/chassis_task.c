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

#include <chassis_task.h>
#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "judge.h"
#include "keyboard.h"
#include "chassis_task.h"
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
        (float vx, float vy, float w_degree);

static void chassis_encoder_update
        (volatile ChassisEncoder_canStruct *encoderPtr);

static void chassis_drive_motor();

static void chassis_state_machine();

/*
 * private parameters
 */
bool            rotation_center_gimbal = false;
float           motor_speed_cutoff_freq = 20.0f;
lpfilterStruct  lp_motor_speed[CHASSIS_MOTOR_NUM];

/**
 * 1. Chassis thread
 */
static THD_WORKING_AREA(chassis_control_wa, 2048);

static THD_FUNCTION(chassis_control, p)
{
    (void) p;
    chRegSetThreadName("chassis controller");

    volatile ChassisEncoder_canStruct* encoderPtr = can_getChassisMotor();

    chassis.ctrl_mode = CHASSIS_STOP;

    uint32_t tick = chVTGetSystemTimeX();

    while (!chThdShouldTerminateX()) {
        tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX()) {
            chThdSleepUntil(tick);
        } else {
            tick = chVTGetSystemTimeX();
        }
        chassis_encoder_update(encoderPtr);

        chassis_state_machine();

        float vx = 0.0f;
        float vy = 0.0f;
        float vw = 0.0f;

        mecanum_inverse_kinematics(vx, vy, vw);

        chassis_drive_motor();
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
        lpfilter_init(lp_motor_speed + i, CHASSIS_UPDATE_FREQ, motor_speed_cutoff_freq);
    }
    chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa), NORMALPRIO,
                      chassis_control, NULL);
}

/**
 * 2. Mechanum wheel kinematics

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
      [//]-------[\\]
           | |
           | |
           | |
      [\\]-------[//]
 */

/*
 * Inverse Kinematics of the four Mecanum wheel system
 * @param chassisPtr
 * @param vx, 3300 ~ -3300
 * @param vy, 3300 ~ -3300
 * @param w,  degree/s
 * @output w1, w2, w3, w4, RPM
 */
static void
mecanum_inverse_kinematics(float vx, float vy, float w_degree)
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
    float vw = w_degree / 6.0f; // Convert the degree/s to RPM

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

    chassis._motors[FRONT_RIGHT].speed_sp = (-1 * vx + vy + vw * rotate_ratio_fr) / r;
    chassis._motors[FRONT_LEFT].speed_sp  = (vx + vy + vw * rotate_ratio_fl) / r;
    chassis._motors[BACK_LEFT].speed_sp   = (vx - vy + vw * rotate_ratio_bl) / r;
    chassis._motors[BACK_RIGHT].speed_sp  = (-1 * vx - vy + vw * rotate_ratio_br) / r;
}

/*
 * Update the motor speed to encoder speed in RPM
 * Change the motor direction to fit the defined orientation
 */
static void
chassis_encoder_update(volatile ChassisEncoder_canStruct *encoderPtr)
{
    for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i) {
        if (encoderPtr[i].updated) {
            encoderPtr[i].updated = false;

            float raw_speed = encoderPtr[i].raw_speed * MOTOR_GEAR_RATIO;
            chassis._motors[i]._speed = lpfilter_apply(&lp_motor_speed[i], raw_speed);
            chassis._motors[i]._wait_count = 0;
            // TODO: change the motor sequence
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

static void
chassis_drive_motor()
{
    can_motorSetCurrent(CHASSIS_CAN_PORT, CHASSIS_CAN_SID,
                        chassis.current[FRONT_RIGHT],
                        chassis.current[FRONT_LEFT],
                        chassis.current[BACK_LEFT],
                        chassis.current[BACK_RIGHT]);
}

/**
 * 3. Chassis control policy
 */


/**
 * 4. State Machine
 */
static void
chassis_state_machine(){

}

