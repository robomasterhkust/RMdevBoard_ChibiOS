/*
 * chassis.c
 *
 *  Updated on: 17 June, 2018
 *      Author: ZENG, Kuang; Tjian
 *      Update: Beck Pang
 */
#include "ch.h"
#include "hal.h"

//#include "canBusProcess.h"
#include "gimbal.h"
#include "dbus.h"
#include "chassis.h"
#include "math_misc.h"
#include "math.h"
#include "keyboard.h"
#include "judge.h"
#include "string.h"
#include "roboconf.h"

static volatile chassisStruct chassis;
volatile GimbalEncoder_canStruct *gimbal_p;
RC_Ctl_t *Rc;
judge_fb_t *JudgeP;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
pid_controller_t chassis_heading_controller;
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];
rc_ctrl_t rm;
static volatile ROS_Msg_Struct *ros_msg;

volatile Gimbal_Send_Dbus_canStruct *pRC;
float gimbal_initP = 0;
float record = 0;

uint32_t twist_count;

#define TWIST_ANGLE 135.0f
#define TWIST_PERIOD 800.0f

#define TWIST_MOVE_ANGLE 90
#define TWIST_MOVE_PERIOD 1000

#define accl_value 0.165f // 165.0/(2*500) // 500 is the frequency and 1 means 1 second
#define accl_y 2.64f // 3300*0.4/(500)
#define accl_x 2.64f // 3300*0.4/(500) //slide

chassis_error_t chassis_getError(void)
{
    return chassis.errorFlag;
}

volatile chassisStruct *chassis_get(void)
{
    return &chassis;
}

#define   CHASSIS_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   CHASSIS_SPEED_PSC 1.0f/((float)CHASSIS_GEAR_RATIO)
#define   CHASSIS_CONNECTION_ERROR_COUNT  20U

static void chassis_encoderUpdate(void)
{
    uint8_t i;
    for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        if (chassis._encoders[i].updated) {
            //Check validiaty of can connection
            chassis._encoders[i].updated = false;

            //float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;
            float speed_input = chassis._encoders[i].raw_speed * CHASSIS_SPEED_PSC;
            chassis._motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
            chassis._motors[i]._wait_count = 1;

            //float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;

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

static int16_t chassis_controlSpeed(motorStruct *motor, pi_controller_t *controller)
{
//  float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_SPEED_PSC);
    float error = motor->speed_sp - motor->_speed;//*wheel_rpm_ratio;
    controller->error_int += error * controller->ki;
    controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
    float output = error * controller->kp + controller->error_int;
    return (int16_t) (boundOutput(output, OUTPUT_MAX));
}

float CHASSIS_RC_MAX_SPEED_X = 3300.0f;
float CHASSIS_RC_MAX_SPEED_Y = 3300.0f;
float CHASSIS_RC_MAX_SPEED_R = 300.0f;
float RC_RESOLUTION = 660.0f;

static void rm_chassis_process(void)
{
    rm.vx = ((pRC->channel0 - RC_CH_VALUE_OFFSET) / RC_RESOLUTION) * CHASSIS_RC_MAX_SPEED_X;
    rm.vy = ((pRC->channel1 - RC_CH_VALUE_OFFSET) / RC_RESOLUTION) * CHASSIS_RC_MAX_SPEED_Y;
}

/*
 * private function used for both normal operation and visual tracking
 * Y points the front, X points right
 */
static void follow_gimbal_handle_private(float vy, float vx)
{
    if (twist_count != 0) {
        twist_count = 0;
    }

    float angle = (gimbal_p[0].radian_angle - gimbal_initP) * 0.66667f;
    if (angle > 0) {
        chassis.drive_sp = 0.8f * (vy * cosf(angle) + vx * sinf(angle));
        chassis.strafe_sp = 0.8f * ((-1) * vy * sinf(angle) + vx * cosf(angle));
    } else {
        chassis.drive_sp = 0.8f * (vy * cosf(-angle) + (-1) * vx * sinf(-angle));
        chassis.strafe_sp = 0.8f * (vy * sinf(-angle) + vx * cosf(-angle));
    }
//    chassis.rotate_sp = chassis_heading_control(&chassis_heading_controller, gimbal_p[0].radian_angle, gimbal_initP);
    chassis.rotate_sp = chassis_heading_control(gimbal_p[0].radian_angle, gimbal_initP);
}

static void visual_follow_gimbal_handle(void) {
    follow_gimbal_handle_private(km.vy + rm.vy + (float)ros_msg->vx * 1000, km.vx + rm.vx);
}

static void follow_gimbal_handle(void) {
    follow_gimbal_handle_private(km.vy + rm.vy, km.vx + rm.vx);
}

static void drive_motor(void)
{
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
                        chassis.current[FRONT_RIGHT], chassis.current[FRONT_LEFT], chassis.current[BACK_LEFT],
                        chassis.current[BACK_RIGHT]); //BR,FR,--,--
}

static THD_WORKING_AREA(chassis_control_wa, 2048);

static THD_FUNCTION(chassis_control, p)
{
    (void) p;
    chRegSetThreadName("chassis controller");

    pRC = can_get_sent_dbus();
    JudgeP = judgeDataGet();
    gimbal_p = can_getGimbalMotor();
    Rc = RC_get();
    ros_msg = can_get_ros_msg();
    bool done = false;
    uint32_t tick = chVTGetSystemTimeX();
    unsigned long tick_test = 0;
    chassis.ctrl_mode = CHASSIS_STOP;
    while (!chThdShouldTerminateX()) {
        // Real time control
        tick_test = ST2US(chVTGetSystemTimeX());
        tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX()) {
            chThdSleepUntil(tick);
            chassis.over_time = false;
        } else {
            tick = chVTGetSystemTimeX();
            chassis.over_time = true;
        }


        if (pRC->updated && !done) {
            done = true;
            chassis.position_ref = gimbal_p[0].radian_angle;
            gimbal_initP = gimbal_p[0].radian_angle;
            pRC->updated = false;
        }
        chassis_encoderUpdate();

        // State Machine to change state
        if (pRC->channel0 > RC_CH_VALUE_MAX || pRC->channel0 < RC_CH_VALUE_MIN) {
            chassis.ctrl_mode = CHASSIS_STOP;
        } else if (judge_is_powered() && JudgeP->powerInfo.powerBuffer <= 20) {
            chassis.ctrl_mode = SAVE_LIFE;
            int i;
            for (i = 0; i < 4; i++) {
                motor_vel_controllers[i].error_int = 0;
            }
        } else if (keyboard_enable((Gimbal_Send_Dbus_canStruct *) pRC)) {
            chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
            keyboard_chassis_process((chassisStruct *) &chassis, (Gimbal_Send_Dbus_canStruct *) pRC);
            rm.vx = 0;
            rm.vy = 0;
        } else if (visual_enable((Gimbal_Send_Dbus_canStruct*) pRC)) {
            chassis.ctrl_mode = AUTO_FOLLOW_GIMBAL;
            rm_chassis_process();
            keyboard_reset();
        } else {
            chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
            rm_chassis_process();
            keyboard_reset();
        }

        switch (chassis.ctrl_mode) {
            case DODGE_MODE:
                chassis.strafe_sp = 0;
                chassis.drive_sp = 0;
                chassis_twist_handle();
                break;
            case AUTO_FOLLOW_GIMBAL:
                visual_follow_gimbal_handle();
                break;
            case AUTO_SEPARATE_GIMBAL:
                break;
            case CHASSIS_STOP:
                chassis_stop_handle();
                break;
            case MANUAL_SEPARATE_GIMBAL:
                separate_gimbal_handle();
                break;
            case MANUAL_FOLLOW_GIMBAL:
                follow_gimbal_handle();
                break;
            case DODGE_MOVE_MODE:
                dodge_move_handle();
                break;
            case SAVE_LIFE:
                if (judge_is_powered()) {
                    save_life();
                }
                break;
            default:
                chassis_stop_handle();
                break;
        }

        mecanum_cal();
        /*
        if (judge_is_powered()) {
            power_limit_handle();
        }
         */
        drive_motor();
        chassis.loop_time = ST2US(chVTGetSystemTimeX()) - tick_test;
    }
}

static const char *FRvelName = "FR_vel";
static const char *FLvelName = "FL_vel";
static const char *BLvelName = "BL_vel";
static const char *BRvelName = "BR_vel";
static const char *HeadingName = "Heading";

#define MOTOR_VEL_INT_MAX 12000U

void chassis_init(void)
{
    memset((void *) &chassis, 0, sizeof(chassisStruct));
    memset((void *) &rm, 0, sizeof(rc_ctrl_t));
    chassis.drive_sp = 0.0f;
    chassis.strafe_sp = 0.0f;
    chassis.rotate_sp = 0.0f;
    chassis.heading_sp = 0.0f;
    chassis.rotate_x_offset = GIMBAL_X_OFFSET;
    chassis.rotate_y_offset = GIMBAL_Y_OFFSET;
    uint8_t i;
    // *********************temporary section*********************
    {
        int j;
        for (j = 0; j < 4; j++) {
            motor_vel_controllers[j].error_int = 0.0f;
            motor_vel_controllers[j].error_int_max = 0.0f;
            motor_vel_controllers[j].ki = 0.0f;
            motor_vel_controllers[j].kp = 100.0f;
        }
    }
    for (i = 0; i < 3; i++) {
        chassis_heading_controller.error[i] = 0.0f;
    }
    chassis_heading_controller.error_int = 0.0f;

    chassis_heading_controller.error_int_max = 0.0f;
    chassis_heading_controller.ki = 0.0f;
    chassis_heading_controller.kp = 70.0f;
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
        chassis._motors[i].speed_curve = 0.0f;

        lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
        motor_vel_controllers[i].error_int = 0.0f;
        motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
    }
    chassis._encoders = can_getChassisMotor();
    chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                      NORMALPRIO, chassis_control, NULL);
}


void mecanum_cal(void)
{
    static float rotate_ratio_fr;
    static float rotate_ratio_fl;
    static float rotate_ratio_br;
    static float rotate_ratio_bl;
    static float wheel_rpm_ratio;
    chSysLock(); // ensure that the calculation is done in batch

/*
    if (chassis.ctrl_mode == DODGE_MODE) {
        chassis.rotate_x_offset = GIMBAL_X_OFFSET;
        chassis.rotate_y_offset = 0;
    } else {
        chassis.rotate_x_offset = 120; //glb_struct.gimbal_x_offset;
        chassis.rotate_y_offset = 0; //glb_struct.gimbal_y_offset;
    }
*/

    if (!chThdShouldTerminateX()) //rotation_center_gimbal
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
    VAL_LIMIT(chassis.rotate_sp, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    /**
     * Add smooth velocity
     */
    if (fabsf(chassis.strafe_curve) < fabsf(chassis.strafe_sp)) {
        if (chassis.strafe_sp > 0) {
            chassis.strafe_curve += accl_x;
        } else {
            chassis.strafe_curve -= accl_x;
        }

        if (fabsf(chassis.strafe_curve) >= fabsf(chassis.strafe_sp)) {
            chassis.strafe_curve = chassis.strafe_sp;
        }
    } else {
        chassis.strafe_curve = chassis.strafe_sp;
    }


    if (fabsf(chassis.drive_curve) < fabsf(chassis.drive_sp)) {
        if (chassis.drive_sp > 0) {
            chassis.drive_curve += accl_y;
        } else {
            chassis.drive_curve -= accl_y;
        }

        if (fabsf(chassis.drive_curve) >= fabsf(chassis.drive_sp)) {
            chassis.drive_curve = chassis.drive_sp;
        }
    } else {
        chassis.drive_curve = chassis.drive_sp;
    }


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
        if (fabsf(chassis._motors[i].speed_sp) > max) {
            max = fabsf(chassis._motors[i].speed_sp);
        }
    }
    //equal proportion
    if (max > MAX_WHEEL_RPM) {
        float rate = MAX_WHEEL_RPM / max;
        int i;
        for (i = 0; i < 4; i++) {
            chassis._motors[i].speed_sp *= rate;
        }
    }
    if (judge_is_powered()) {
        speed_limit_handle();
    }

    for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
        chassis.current[i] = chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);
        VAL_LIMIT(chassis.current[i], -16384, 16384);
        //chassis.current[i] = 0;
    }
}

void chassis_twist_handle()
{
    float twist_period = TWIST_PERIOD;
    float twist_angle = TWIST_ANGLE;
    twist_count++;
    chassis.position_ref = gimbal_initP +
                           twist_angle * sinf(2.0f * (float) M_PI / twist_period * twist_count) * (float) M_PI / 180.0f;
    /*Develop later
     * */
//    chassis.rotate_sp = chassis_heading_control(&chassis_heading_controller, gimbal_p[0].radian_angle,
//                                                chassis.position_ref);
    chassis.rotate_sp = chassis_heading_control(gimbal_p[0].radian_angle, chassis.position_ref);
}


void dodge_move_handle()
{
    float twist_period = TWIST_MOVE_PERIOD;
    float twist_angle = TWIST_MOVE_ANGLE;
    twist_count++;
    chassis.position_ref = gimbal_initP +
                           twist_angle * sinf(2.0f * (float) M_PI / twist_period * twist_count) * (float) M_PI / 180.0f;
//    chassis.rotate_sp = chassis_heading_control(&chassis_heading_controller, gimbal_p[0].radian_angle,
//                                                chassis.position_ref);
    chassis.rotate_sp = chassis_heading_control(gimbal_p[0].radian_angle, chassis.position_ref);
    float vy = km.vy;
    float vx = km.vx;
    float angle = (gimbal_p[0].radian_angle - gimbal_initP) * 0.66667f;
    if (angle > 0) {
        chassis.drive_sp = 0.8f * (vy * cosf(angle) + vx * sinf(angle));
        chassis.strafe_sp = 0.8f * ((-1) * vy * sinf(angle) + vx * cosf(angle));
    } else {
        chassis.drive_sp = 0.8f * (vy * cosf(-angle) + (-1) * vx * sinf(-angle));
        chassis.strafe_sp = 0.8f * (vy * sinf(-angle) + vx * cosf(-angle));
    }
}

void chassis_stop_handle()
{
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
void separate_gimbal_handle()
{
    chassis.drive_sp = rm.vy + km.vy;
    chassis.strafe_sp = rm.vx + km.vx;
    chassis.rotate_sp = rm.vw;
}
/*
float chassis_heading_control(pid_controller_t *controller, float get, float set)
{
    controller->error[0] = set - get;
    float output = controller->error[0] * controller->kp +
                   controller->kd * (controller->error[0] - 2 * controller->error[1] + controller->error[2]);
    controller->error[1] = controller->error[0];
    controller->error[2] = controller->error[1];

    return output;
}
*/

float chassis_heading_control(float get, float set)
{
    chassis_heading_controller.error[0] = set - get;
    float output =  chassis_heading_controller.error[0] * chassis_heading_controller.kp +
                    chassis_heading_controller.kd * (chassis_heading_controller.error[0] -
                    2 * chassis_heading_controller.error[1] + chassis_heading_controller.error[2]);
    chassis_heading_controller.error[1] = chassis_heading_controller.error[0];
    chassis_heading_controller.error[2] = chassis_heading_controller.error[1];

    return output;
}

void speed_limit_handle()
{
    if (JudgeP->powerInfo.power > CHASSIS_POWER_MAX_W) {
        if (JudgeP->powerInfo.powerBuffer <= CHASSIS_POWER_BUFFER_J * 0.66667f) {
            int i;
            for (i = 0; i < 4; ++i) {
                if (chassis._motors[i].speed_sp >= chassis._motors[i]._speed &&
                    fabsf(chassis._motors[i].speed_sp) >= fabsf(chassis._motors[i]._speed)) {
                    chassis._motors[i].speed_curve = chassis._motors[i]._speed + accl_value;
                    if (chassis._motors[i].speed_curve < chassis._motors[i].speed_sp) {
                        chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
                    }
                } else if (chassis._motors[i].speed_sp <= chassis._motors[i]._speed &&
                           fabsf(chassis._motors[i].speed_sp) >= fabsf(chassis._motors[i]._speed)) {
                    chassis._motors[i].speed_curve = chassis._motors[i]._speed - accl_value;
                    if (chassis._motors[i].speed_curve < chassis._motors[i].speed_sp) {
                        chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
                    }
                } else {
                    chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
                }

            }
        } else {
            int i;
            for (i = 0; i < 4; i++) {
                if (chassis._motors[i].speed_sp > chassis._motors[i].speed_curve &&
                    fabsf(chassis._motors[i].speed_sp) > fabsf(chassis._motors[i].speed_curve)) {
                    chassis._motors[i].speed_curve += accl_value;
                } else if (chassis._motors[i].speed_sp < chassis._motors[i].speed_curve &&
                           fabsf(chassis._motors[i].speed_sp) > fabsf(chassis._motors[i].speed_curve)) {
                    chassis._motors[i].speed_curve -= accl_value;
                } else {
                    chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
                }

                if (fabsf(chassis._motors[i].speed_curve) >= fabsf(chassis._motors[i].speed_sp)) {
                    chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
                }
            }
        }

    } else {
        int i;
        for (i = 0; i < 4; i++) {
            chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
        }
    }
}

void save_life()
{
    chassis.strafe_curve = 0;
    chassis.drive_curve = 0;
    chassis.rotate_sp = 0;
    chassis.strafe_sp = 0;
    chassis.drive_sp = 0;

    if (JudgeP->powerInfo.powerBuffer >= CHASSIS_POWER_BUFFER_J * 0.83333f) {
        chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
    }
}

void power_limit_handle()
{
    double power_limit = CHASSIS_POWER_MAX_W + 0.5 * JudgeP->powerInfo.powerBuffer;
    double a0 = fabs(chassis._encoders[0].act_current * 20.0 / 16384.0);
    double a1 = fabs(chassis._encoders[1].act_current * 20.0 / 16384.0);
    double a2 = fabs(chassis._encoders[2].act_current * 20.0 / 16384.0);
    double a3 = fabs(chassis._encoders[3].act_current * 20.0 / 16384.0);
    double MAX = (power_limit - 3 - 0.14 * (a0 * a0 + a1 * a1 + a2 * a2 + a3 * a3)) / 0.005;


    double A0 = fabs(chassis.current[0] * 20.0 / 16384.0);
    double A1 = fabs(chassis.current[1] * 20.0 / 16384.0);
    double A2 = fabs(chassis.current[2] * 20.0 / 16384.0);
    double A3 = fabs(chassis.current[3] * 20.0 / 16384.0);
    double SUM = fabs(chassis._encoders[0].raw_speed) * A0
                 + fabs(chassis._encoders[1].raw_speed) * A1
                 + fabs(chassis._encoders[2].raw_speed) * A2
                 + fabs(chassis._encoders[3].raw_speed) * A3;

    if (SUM > MAX && JudgeP->powerInfo.power >= CHASSIS_POWER_MAX_W) {
        chassis.current[FRONT_RIGHT] = (int16_t) (chassis.current[FRONT_RIGHT] * MAX / SUM);
        chassis.current[FRONT_LEFT] = (int16_t) (chassis.current[FRONT_LEFT] * MAX / SUM);
        chassis.current[BACK_LEFT] = (int16_t) (chassis.current[BACK_LEFT] * MAX / SUM);
        chassis.current[BACK_RIGHT] = (int16_t) (chassis.current[BACK_RIGHT] * MAX / SUM);
        chassis.over_power = true;
    } else {
        chassis.over_power = false;
    }
}



