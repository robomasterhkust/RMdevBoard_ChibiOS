/*
 * chassis.c
 *
 *  Created on: 06 April, 2018
 *      Author: Tian TJian, Kuang Zeng
 */
#include "ch.h"
#include "hal.h"

//#include "canBusProcess.h"
#include "can_motor_task.h"
#include "gimbal_simple_controller.h"
#include "dbus.h"
#include "chassis.h"
#include "adis16265.h"
#include "math_misc.h"
#include "math.h"
#include "string.h"
#include "gimbal.h"
//#include "keyboard.h"

static chassisStruct chassis;
GimbalStruct *gimbal_p;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
pid_controller_t heading_controller;
pid_controller_t chassis_heading_controller;
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];
rc_ctrl_t rm;
//RC_Ctl_t *pRC;
volatile Gimbal_Send_Dbus_canStruct *pGimbalRC;
int rotation_center_gimbal = 1;
float gimbal_initP = 0;
float record = 0;

#define TWIST_ANGLE 50
#define TWIST_PERIOD 3000


chassis_error_t chassis_getError(void) {
    return chassis.errorFlag;
}

chassisStruct *chassis_get(void) {
    return &chassis;
}

#define   CHASSIS_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   CHASSIS_SPEED_PSC 1.0f/((float)CHASSIS_GEAR_RATIO)
#define   CHASSIS_CONNECTION_ERROR_COUNT  20U

static void chassis_encoderUpdate(void) {
    uint8_t i;
    for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (chassis._encoders[i].updated) {
            //Check validiaty of can connection
            chassis._encoders[i].updated = false;

            //float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;
            float speed_input = chassis._encoders[i].raw_speed * CHASSIS_SPEED_PSC;
            chassis._motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
            chassis._motors[i]._wait_count = 1;
        } else {
            chassis._motors[i]._wait_count++;
            if (chassis._motors[i]._wait_count > CHASSIS_CONNECTION_ERROR_COUNT) {
                chassis.errorFlag |= CHASSIS_MOTOR_NOT_CONNECTED << i;
                chassis._motors[i]._wait_count = 1;
            }
        }
    }
#ifdef CHASSIS_USE_POS_MOTOR
#endif
}

#define OUTPUT_MAX  16384

static int16_t chassis_controlSpeed(motorStruct *motor, pi_controller_t *controller) {
//  float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_SPEED_PSC);
    float error = motor->speed_sp - motor->_speed;//*wheel_rpm_ratio;
    controller->error_int += error * controller->ki;
    controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
    float output = error * controller->kp + controller->error_int;
    return (int16_t) (boundOutput(output, OUTPUT_MAX));
}


#define H_MAX  200  // Heading PID_outputx

static int16_t chassis_controlHeading(chassisStruct *chassis, pid_controller_t *controller) {
    float error = chassis->heading_sp - chassis->_pGyro->angle;
    controller->error_int += error * controller->ki;
    controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
    float output = error * controller->kp + controller->error_int + controller->kd * (error - chassis->pid_last_error);
    chassis->pid_last_error = error;
    return (int16_t) (boundOutput(output, H_MAX));
}

float CHASSIS_RC_MAX_SPEED_X = 3300.0f;
float CHASSIS_RC_MAX_SPEED_Y = 3300.0f;
float CHASSIS_RC_MAX_SPEED_R = 300.0f;
float RC_RESOLUTION = 660.0f;

static void rm_chassis_process(void) {
    rm.vx = (pGimbalRC->channel0 - 1024) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
    rm.vy = (pGimbalRC->channel1 - 1024) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
}

static THD_WORKING_AREA(chassis_control_wa, 2048);

static THD_FUNCTION(chassis_control, p) {

    (void) p;
    chRegSetThreadName("chassis controller");

//    pRC = RC_get();
    pGimbalRC = can_get_sent_dbus();
//    gimbal_p = gimbal_get();
    gimbal_p = get_gimbal_simple_controller();
    bool done = false;
    uint32_t tick = chVTGetSystemTimeX();
    chassis.ctrl_mode = CHASSIS_STOP;
    while (!chThdShouldTerminateX()) {
        if (gimbal_p->inited) {

            chassis.position_ref = gimbal_p->yaw_angle;
            gimbal_initP = gimbal_p->yaw_angle;
            chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
        }

        tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else {
            tick = chVTGetSystemTimeX();
        }
        chassis_encoderUpdate();
        rm_chassis_process();

        //if(keyboard_enable()){
        //    keyboard_chassis_process(&chassis);
        //    rm.vx = 0;
        //    rm.vy = 0;
        //}
        //else{
        //    rm_chassis_process();
        //    keyboard_reset();
        //}
        // keyboardprocess(pRC->key_code);
        switch (chassis.ctrl_mode) {
            case DODGE_MODE: {
                chassis.strafe_sp = 0;
                chassis.drive_sp = 0;
                chassis_twist_handle();
            }
                break;
            case AUTO_FOLLOW_GIMBAL: {
                /*Develop later
                 * */
            }
                break;
            case AUTO_SEPARATE_GIMBAL: {
                /*Develop later
                 * */
            }
                break;
            case CHASSIS_STOP: {
                chassis_stop_handle();
            }
                break;
            case MANUAL_SEPARATE_GIMBAL: {
                separate_gimbal_handle();
            }
                break;
            case MANUAL_FOLLOW_GIMBAL: {
                follow_gimbal_handle();
            }
                break;
            default: {
                chassis_stop_handle();
            }
                break;
        }
        mecanum_cal();
        drive_motor();
    }
}

static const char FRvelName[] = "FR_vel";
static const char FLvelName[] = "FL_vel";
static const char BLvelName[] = "BL_vel";
static const char BRvelName[] = "BR_vel";
static const char HeadingName[] = "Heading";

#define MOTOR_VEL_INT_MAX 12000U

void chassis_init(void) {
    memset((void *) &chassis, 0, sizeof(chassisStruct));
    rotation_center_gimbal = 1;
    chassis.drive_sp = 0.0f;
    chassis.strafe_sp = 0.0f;
    chassis.rotate_sp = 0.0f;
    chassis.heading_sp = 0.0f;
    chassis.rotate_x_offset = 0;
    chassis.rotate_y_offset = 0;
    uint8_t i;
    // *********************temporary section*********************
    {
        int j;
        for (j = 0; j < 4; j++) {
            motor_vel_controllers[j].error_int = 0.0f
            motor_vel_controllers[j].error_int_max = 0.0f;
            motor_vel_controllers[j].ki = 0.0f;
            motor_vel_controllers[j].kp = 5.0f;
        }
    }
    heading_controller.error_int = 0.0f;
    heading_controller.error_int_max = 0.0f;
    heading_controller.ki = 0.0f;
    heading_controller.kp = 0.0f;

    for (i = 0; i < 3; i++) {
        chassis_heading_controller.error[i] = 0.0f;
    }
    chassis_heading_controller.error_int = 0.0f;

    chassis_heading_controller.error_int_max = 0.0f;
    chassis_heading_controller.ki = 0.0f;
    chassis_heading_controller.kp = 20.0f;
    chassis_heading_controller.kd = 1.0f;
    //**************************************************************
//  params_set(&motor_vel_controllers[FRONT_LEFT], 9,2,FLvelName,subname_PI,PARAM_PUBLIC);
//  params_set(&motor_vel_controllers[FRONT_RIGHT], 10,2,FRvelName,subname_PI,PARAM_PUBLIC);
//  params_set(&motor_vel_controllers[BACK_LEFT], 11,2,BLvelName,subname_PI,PARAM_PUBLIC);
//  params_set(&motor_vel_controllers[BACK_RIGHT], 12,2,BRvelName,subname_PI,PARAM_PUBLIC);
//  params_set(&heading_controller, 13, 3, HeadingName,subname_PID,PARAM_PUBLIC);

    for (i = 0; i < 4; i++) {
        chassis.current[i] = 0;
        chassis._motors[i].speed_sp = 0.0f;
        lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
        motor_vel_controllers[i].error_int = 0.0f;
        motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
    }
    heading_controller.error_int = 0.0f;
    heading_controller.error_int_max = 0.0f;
    chassis._pGyro = gyro_get();
    chassis._encoders = can_getChassisMotor();

    chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                      NORMALPRIO, chassis_control, NULL);
}


void update_heading(void) {
    /*TODO THIS WILL MAKE YOU LOSE HEADING IN A COLLISION, add control*/
    chassis.heading_sp = chassis._pGyro->angle;
}

/*
 *
 *
 *
 *
 *
 * */

void mecanum_cal() {
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_br;
    static float rotate_ratio_bl;
    static float wheel_rpm_ratio;
    chSysLock(); // ensure that the calculation is done in batch

    if (chassis.ctrl_mode == DODGE_MODE) {
        chassis.rotate_x_offset = GIMBAL_X_OFFSET;
        chassis.rotate_y_offset = 0;
    } else {
        chassis.rotate_x_offset = 120; //glb_struct.gimbal_x_offset;
        chassis.rotate_y_offset = 0; //glb_struct.gimbal_y_offset;
    }

    if (1) //rotation_center_gimbal
    {
        rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f \
 - chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;
        rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f \
 - chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
        rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f \
 + chassis.rotate_x_offset - chassis.rotate_y_offset) / RADIAN_COEF;
        rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f \
 + chassis.rotate_x_offset + chassis.rotate_y_offset) / RADIAN_COEF;
    } else {
        rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
        rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
        rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
        rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    }

    wheel_rpm_ratio = 60.0f / (PERIMETER);
    chSysUnlock();


//  VAL_LIMIT(chassis.drive_sp, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
//  VAL_LIMIT(chassis.strafe_sp, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
//  VAL_LIMIT(chassis.rotate_sp, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    chassis._motors[FRONT_RIGHT].speed_sp =
            (chassis.strafe_sp - chassis.drive_sp + chassis.rotate_sp * rotate_ratio_fr) *
            wheel_rpm_ratio;   // CAN ID: 0x201
    chassis._motors[BACK_RIGHT].speed_sp =
            (-1 * chassis.strafe_sp - chassis.drive_sp + chassis.rotate_sp * rotate_ratio_br) *
            wheel_rpm_ratio;       // CAN ID: 0x202
    chassis._motors[FRONT_LEFT].speed_sp =
            (chassis.strafe_sp + chassis.drive_sp + chassis.rotate_sp * rotate_ratio_fl) *
            wheel_rpm_ratio;       // CAN ID: 0x203
    chassis._motors[BACK_LEFT].speed_sp =
            (-1 * chassis.strafe_sp + chassis.drive_sp + chassis.rotate_sp * rotate_ratio_bl) *
            wheel_rpm_ratio;     // CAN ID: 0x204
    float max = 0.0f;
    //find max item
    int i;
    for (i = 0; i < 4; i++) {
        if (fabsf(chassis._motors[i].speed_sp) > max)
            max = fabsf(chassis._motors[i].speed_sp);
    }
    //equal proportion
    if (max > MAX_WHEEL_RPM) {
        float rate = MAX_WHEEL_RPM / max;

        for (i = 0; i < 4; i++) {
            chassis._motors[i].speed_sp *= rate;
        }
    }
}

void drive_motor() {

    uint8_t i;
    for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        chassis.current[i] = chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);
    }
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
                        chassis.current[FRONT_RIGHT], chassis.current[FRONT_LEFT], chassis.current[BACK_LEFT],
                        chassis.current[BACK_RIGHT]); //BR,FR,--,--
}

uint32_t twist_count;

void chassis_twist_handle() {
    int16_t twist_period = TWIST_PERIOD;
    int16_t twist_angle = TWIST_ANGLE;
    twist_count++;
    chassis.position_ref =
            gimbal_initP + twist_angle * sinf(2.0f * (float) M_PI / twist_period * twist_count) * (float) M_PI / 180.0f;
    /*Develop later
     * */
    chassis.rotate_sp = chassis_heading_control(&chassis_heading_controller, gimbal_p->yaw_angle, chassis.position_ref);
}

void chassis_stop_handle() {
    chassis.strafe_sp = 0;
    chassis.drive_sp = 0;
    chassis.rotate_sp = 0;
}


/*
static void chassis_operation_func(int16_t left_right, int16_t front_back, int16_t rotate)
{
 rc.vx =  left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
 rc.vy =    front_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
 rc.vw =  rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R;
}
*/
void separate_gimbal_handle() {
    chassis.drive_sp = rm.vy;
    chassis.strafe_sp = rm.vx;
    chassis.rotate_sp = 0;
}

void follow_gimbal_handle() {
    if (twist_count != 0) {
        twist_count = 0;
    }
    float vy = rm.vy;
    float vx = rm.vx;
    float angle = (gimbal_p->yaw_angle - gimbal_initP) * (2.0f / 3.0f);
    if (angle > 0) {
        chassis.drive_sp = (vy * cosf(angle) + vx * sinf(angle));
        chassis.strafe_sp = (-1) * vy * sinf(angle) + vx * cosf(angle);
    } else {
        chassis.drive_sp = (vy * cosf(-angle) + (-1) * vx * sinf(-angle));
        chassis.strafe_sp = vy * sinf(-angle) + vx * cosf(-angle);
    }
    chassis.rotate_sp = chassis_heading_control(&chassis_heading_controller, gimbal_p->yaw_angle, gimbal_initP);
}

float chassis_heading_control(pid_controller_t *controller, float get, float set) {
    controller->error[0] = set - get;
    float output = controller->error[0] * controller->kp +
                   controller->kd * (controller->error[0] - 2 * controller->error[2] + controller->error[3]);
    controller->error[1] = controller->error[0];
    controller->error[2] = controller->error[1];

    return output;

}
