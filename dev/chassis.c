/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
*/
#include "ch.h"
#include "hal.h"
#include <canBusProcess.h>

#include "adis16265.h"
#include "canBusProcess.h"
#include "chassis.h"
#include "dbus.h"
#include "gimbal.h"
#include "judge.h"
#include "keyboard.h"
#include "magazine_cover_task.h"
#include "math.h"
#include "math_misc.h"
static volatile chassisStruct chassis;

GimbalEncoder_canStruct *gimbal_p;
RC_Ctl_t *Rc;
judge_fb_t *JudgeP;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
pid_controller_t chassis_heading_controller;
pid_controller_t dancing_controller;
pid_controller_t power_limit_controller;
pid_controller_t acceleration_limit_controller;
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];
rc_ctrl_t rm;
Gimbal_Send_Dbus_canStruct *pRC;
float gimbal_initP = 0;
float record = 0;
bool reboot = false;

#define TWIST_ANGLE 150
#define TWIST_PERIOD 800

#define TWIST_MOVE_ANGLE 90
#define TWIST_MOVE_PERIOD 1000

#define accl_value 165.0 / (500) // 500 is the frequency and 1 means 1 second
#define accl_y 3300 * 0.4 / (500)
#define accl_x 3300 * 0.4 / (500) // slide
#define deccl_y 3300 / (500)
#define deccl_x 3300 / (500)
chassis_error_t chassis_getError(void) { return chassis.errorFlag; }

chassisStruct *chassis_get(void) { return &chassis; }

#define CHASSIS_ANGLE_PSC 7.6699e-4 // 2*M_PI/0x1FFF
#define CHASSIS_SPEED_PSC 1.0f / ((float)CHASSIS_GEAR_RATIO)
#define CHASSIS_CONNECTION_ERROR_COUNT 20U
static void chassis_encoderUpdate(void) {
  uint8_t i;
  for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    if (chassis._encoders[i].updated) {
      // Check validiaty of can connection
      chassis._encoders[i].updated = false;

      // float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;
      float speed_input = chassis._encoders[i].raw_speed * CHASSIS_SPEED_PSC;
      chassis._motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
      chassis._motors[i]._wait_count = 1;

      // float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;

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
#define OUTPUT_MAX 16384
static int16_t chassis_controlSpeed(motorStruct *motor,
                                    pi_controller_t *controller) {
  //  float wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_SPEED_PSC);

  float error = motor->speed_curve - motor->_speed; //*wheel_rpm_ratio;
  controller->error_int += error * controller->ki;
  controller->error_int =
      boundOutput(controller->error_int, controller->error_int_max);
  float output = error * controller->kp + controller->error_int;
  return (int16_t)(boundOutput(output, OUTPUT_MAX));
}

float acceleration_limit_control(pid_controller_t *controller, float get,
                                 float set) {
  controller->error[0] = set - get;
  controller->error_int += controller->error[0] * controller->ki;
  controller->error_int =
      boundOutput(controller->error_int, controller->error_int_max);
  float output =
      controller->error_int + controller->error[0] * controller->kp +
      controller->kd * (controller->error[0] - 2 * controller->error[1] +
                        controller->error[2]);
  controller->error[1] = controller->error[0];
  controller->error[2] = controller->error[1];
  return boundOutput(output, accl_y);
}


bool chassis_absolute_speed(float i){
  float rpm = fabsf(chassis._motors[0]._speed)/4 + fabsf(chassis._motors[1]._speed)/4 + fabsf(chassis._motors[2]._speed)/4 + fabsf(chassis._motors[3]._speed)/4;
  if(rpm > 125*i){
    return true;
  }
  return false;
}

/*
#define H_MAX  200  // Heading PID_outputx
static int16_t chassis_controlHeading(chassisStruct* chassis, pid_controller_t*
controller)
{
  float error = chassis->heading_sp - chassis->_pGyro->angle;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int,
controller->error_int_max);
  float output = error*controller->kp + controller->error_int + controller->kd *
(error - chassis->pid_last_error);
  chassis->pid_last_error = error;
  return (int16_t)(boundOutput(output, H_MAX));
}
*/

float CHASSIS_RC_MAX_SPEED_X = 3300.0f;
float CHASSIS_RC_MAX_SPEED_Y = 3300.0f;
float CHASSIS_RC_MAX_SPEED_R = 300.0f;
float RC_RESOLUTION = 660.0f;

static void rm_chassis_process(void) {

  rm.vx = (pRC->channel0 - 1024) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
  rm.vy = (pRC->channel1 - 1024) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;

  /*
       rm.vx =  (Rc->rc.channel0 - 1024) / RC_RESOLUTION *
     CHASSIS_RC_MAX_SPEED_X;
       rm.vy =   (Rc->rc.channel1 - 1024) / RC_RESOLUTION *
     CHASSIS_RC_MAX_SPEED_Y;
       rm.vw = (Rc->rc.channel2 - 1024) / RC_RESOLUTION * MAX_CHASSIS_VR_SPEED;
  */
}

static inline void motor_debug_can(CANDriver *const CANx) {
  CANTxFrame txmsg;
  MotorDebug_canStruct motor_debug[CHASSIS_MOTOR_NUM];
  uint8_t i;
  for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    motor_debug[i]._speed = chassis._motors[i]._speed;
    motor_debug[i].speed_curve = chassis._motors[i].speed_curve;
  }
  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = CAN_CHASSIS_DEBUG_FR;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();
  memcpy(&(txmsg.data8), &motor_debug[0], 8);
  chSysUnlock();
  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

  txmsg.SID = CAN_CHASSIS_DEBUG_FL;
  chSysLock();
  memcpy(&(txmsg.data8), &motor_debug[1], 8);
  chSysUnlock();
  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

  txmsg.SID = CAN_CHASSIS_DEBUG_BL;
  chSysLock();
  memcpy(&(txmsg.data8), &motor_debug[2], 8);
  chSysUnlock();
  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

  txmsg.SID = CAN_CHASSIS_DEBUG_BR;
  chSysLock();
  memcpy(&(txmsg.data8), &motor_debug[3], 8);
  chSysUnlock();
  canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static THD_WORKING_AREA(chassis_can_Thd_wa, 1024);
static THD_FUNCTION(chassis_can_Thd, p) {
  (void)p;
  while (!chThdShouldTerminateX()) {
    motor_debug_can(MOTOR_DEBUG_CAN);
    chThdSleep(CHASSIS_CAN_UPDATE_PERIOD);
  }
}

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p) {
  (void)p;
  chRegSetThreadName("chassis controller");

  pRC = can_get_sent_dbus();
  JudgeP = judgeDataGet();
  gimbal_p = can_getGimbalMotor();
  Rc = RC_get();
  uint32_t tick = chVTGetSystemTimeX();
  //  uint32_t tick_magazine = ST2MS(chVTGetSystemTimeX());
  chassis.ctrl_mode = CHASSIS_STOP;
  while (!chThdShouldTerminateX()) {

    if (pRC->channel0 > 1684 || pRC->channel0 < 0) {
      chassis.ctrl_mode = CHASSIS_STOP;
      reboot = false;
    } else {
      chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
      if (!reboot) {
        reboot = true;
        chassis.position_ref = gimbal_p[0].radian_angle;
        gimbal_initP = gimbal_p[0].radian_angle;
        chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
      }
    }

    //   tick_magazine = ST2US(chVTGetSystemTimeX());
    tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
    if (tick > chVTGetSystemTimeX()) {
      chThdSleepUntil(tick);
      chassis.over_time = false;
    } else {
      tick = chVTGetSystemTimeX();
      chassis.over_time = true;
    }
    chassis_encoderUpdate();
/*
    if(JudgeP->powerInfo.powerBuffer<=5){
      int i;
      for(i =0; i<4;i++){
        motor_vel_controllers[i].error_int = 0;
      }
      chassis.ctrl_mode = SAVE_LIFE;
    }
*/
    chassis.power_limit = 1.3*JudgeP->powerInfo.powerBuffer;
    if(JudgeP->powerInfo.powerBuffer<=30){
      chassis.power_limit = 2.5*JudgeP->powerInfo.powerBuffer;
      if(chassis.power_limit <= 25 && JudgeP->powerInfo.power > 80 && !chassis_absolute_speed(1)){


        chassis.ctrl_mode = SAVE_LIFE;
        // chassis.power_limit = 0;
        int i;
        for (i = 0; i < 4; i++) {
          motor_vel_controllers[i].error_int = 0;
        }
        power_limit_controller.error_int = 0;
      }
    } else {
      chassis.power_limit = 80;
    }

    if (keyboard_enable(pRC)) {
      keyboard_chassis_process(&chassis, pRC);
      rm.vx = 0;
      rm.vy = 0;
    } else {
      rm_chassis_process();
      keyboard_reset();
    }
    switch (chassis.ctrl_mode) {
    case DODGE_MODE: {
      chassis.strafe_sp = 0;
      chassis.drive_sp = 0;
      chassis_twist_handle();
    } break;
    case AUTO_FOLLOW_GIMBAL: {
      /*Develop later
       * */
    } break;
    case AUTO_SEPARATE_GIMBAL: {
      /*Develop later
       * */
    } break;
    case CHASSIS_STOP: {
      chassis_stop_handle();
    } break;
    case MANUAL_SEPARATE_GIMBAL: {
      separate_gimbal_handle();
    } break;
    case MANUAL_FOLLOW_GIMBAL: {
      follow_gimbal_handle();
    } break;
    case DODGE_MOVE_MODE: {
      dodge_move_handle();
    } break;
    case SAVE_LIFE: {
      save_life();
    } break;
    default: { chassis_stop_handle(); } break;
    }
    mecanum_cal();
    drive_motor();
  }
}

static const FRvelName = "FR_vel";
static const FLvelName = "FL_vel";
static const BLvelName = "BL_vel";
static const BRvelName = "BR_vel";
static const HeadingName = "Heading";

#define MOTOR_VEL_INT_MAX 12000U
void chassis_init(void) {
  memset(&chassis, 0, sizeof(chassisStruct));
  chassis.drive_sp = 0.0f;
  chassis.drive_curve = 0.0f;
  chassis.strafe_curve = 0.0f;
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
      motor_vel_controllers[j].ki = 0.2f;
      motor_vel_controllers[j].kp = 150.0f; // 100
    }
  }
  for (i = 0; i < 3; i++) {
    chassis_heading_controller.error[i] = 0.0f;
    dancing_controller.error[i] = 0.0f;
  }
  chassis_heading_controller.error_int = 0.0f;

  chassis_heading_controller.error_int_max = 0.0f;
  chassis_heading_controller.ki = 0.0f;
  chassis_heading_controller.kp = 70.0f;
  chassis_heading_controller.kd = 1.0f;

  power_limit_controller.error_int = 0.0f;
  power_limit_controller.error_int_max = 0.0f;
  power_limit_controller.ki = 0.0f;
  power_limit_controller.kp = 10.0f;
  power_limit_controller.kd = 0.0f;

  acceleration_limit_controller.error_int = 0.0f;
  acceleration_limit_controller.error_int_max = accl_x;
  acceleration_limit_controller.ki = 0.00005f;
  acceleration_limit_controller.kp = 0.03f;
  acceleration_limit_controller.kd = 0.018f;

  dancing_controller.error_int = 0.0f;
  dancing_controller.error_int_max = 200.0f;
  dancing_controller.ki = 0.1f;
  dancing_controller.kp = 70.0f;
  dancing_controller.kd = 1.0f;
  //**************************************************************

  //  params_set(&motor_vel_controllers[FRONT_LEFT],
  //  9,2,FLvelName,subname_PI,PARAM_PUBLIC);
  //  params_set(&motor_vel_controllers[FRONT_RIGHT],
  //  10,2,FRvelName,subname_PI,PARAM_PUBLIC);
  //  params_set(&motor_vel_controllers[BACK_LEFT],
  //  11,2,BLvelName,subname_PI,PARAM_PUBLIC);
  //  params_set(&motor_vel_controllers[BACK_RIGHT],
  //  12,2,BRvelName,subname_PI,PARAM_PUBLIC);
  //  params_set(&heading_controller, 13, 3,
  //  HeadingName,subname_PID,PARAM_PUBLIC);

  for (i = 0; i < 4; i++) {
    chassis.current[i] = 0;
    chassis._motors[i].speed_sp = 0.0f;
    chassis._motors[i].speed_curve = 0.0f;

    lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
    motor_vel_controllers[i].error_int = 0.0f;
    motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
  }
  chassis._pGyro = gyro_get();
  chassis._encoders = can_getExtraMotor();
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa), NORMALPRIO,
                    chassis_control, NULL);
  chThdCreateStatic(chassis_can_Thd_wa, sizeof(chassis_can_Thd_wa), NORMALPRIO,
                    chassis_can_Thd, NULL);
}

void mecanum_cal(){
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_br;
  static float rotate_ratio_bl;
  static float wheel_rpm_ratio;
  chSysLock(); // ensure that the calculation is done in batch

  if (1) // rotation_center_gimbal
  {
    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f -
                       chassis.rotate_x_offset + chassis.rotate_y_offset) /
                      RADIAN_COEF;
    rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f -
                       chassis.rotate_x_offset - chassis.rotate_y_offset) /
                      RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f +
                       chassis.rotate_x_offset - chassis.rotate_y_offset) /
                      RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f +
                       chassis.rotate_x_offset + chassis.rotate_y_offset) /
                      RADIAN_COEF;
  } else {
    rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
  }

  wheel_rpm_ratio = 60.0f / (PERIMETER);
  chSysUnlock();

  //  VAL_LIMIT(chassis.drive_sp, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
  //  //mm/s
  //  VAL_LIMIT(chassis.strafe_sp, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
  //  //mm/s
  VAL_LIMIT(chassis.rotate_sp, -MAX_CHASSIS_VR_SPEED,
            MAX_CHASSIS_VR_SPEED); // deg/s

  //  if(JudgeP->powerInfo.powerBuffer<=40 && reboot){
  //    power_limit_handle();
  //  }
  //  else{
  if (fabs(chassis.strafe_curve) < fabs(chassis.strafe_sp)) {
    if (chassis.strafe_sp >= 0) {
      chassis.strafe_curve += acceleration_limit_control(
          &acceleration_limit_controller, JudgeP->powerInfo.power,
          chassis.power_limit);
      if (chassis.strafe_curve <= 0) {
        chassis.strafe_curve = 0;
      }
    } else {
      chassis.strafe_curve -= acceleration_limit_control(
          &acceleration_limit_controller, JudgeP->powerInfo.power,
          chassis.power_limit);
      if (chassis.strafe_curve >= 0) {
        chassis.strafe_curve = 0;
      }
    }

    if (fabs(chassis.strafe_curve) >= fabs(chassis.strafe_sp)) {
      chassis.strafe_curve = chassis.strafe_sp;
    }

  } else if (fabs(chassis.strafe_curve) > fabs(chassis.strafe_sp)) {
    // if(fabs(chassis.strafe_sp) < 0.003){ // check whether the user intended
    // to stop
    float previous_strafe_curve = chassis.strafe_curve;
    if (chassis.strafe_curve >= 0) {
      chassis.strafe_curve -= deccl_y;
    } else {
      chassis.strafe_curve += deccl_y;
    }

    if (fabs(chassis.strafe_sp) < 0.003) {
      if (((previous_strafe_curve > 0) & (chassis.strafe_curve < 0)) |
          ((previous_strafe_curve < 0) & (chassis.strafe_curve > 0))) {
        chassis.strafe_curve = 0;
      }
    } else if (fabs(chassis.strafe_curve) < fabs(chassis.strafe_sp)) {
      chassis.strafe_curve = chassis.strafe_sp;
    } else {
      chassis.strafe_curve = chassis.strafe_sp;
    }
  } else {
    chassis.strafe_curve = chassis.strafe_sp;
  }

  if (fabs(chassis.drive_curve) < fabs(chassis.drive_sp)) {
    if (chassis.drive_sp >= 0) {
      chassis.drive_curve += acceleration_limit_control(
          &acceleration_limit_controller, JudgeP->powerInfo.power, chassis.power_limit);
      if (chassis.drive_curve <= 0) {
        chassis.drive_curve = 0;
      }
    } else {
      chassis.drive_curve -= acceleration_limit_control(
          &acceleration_limit_controller, JudgeP->powerInfo.power, chassis.power_limit);
      if (chassis.drive_curve >= 0) {
        chassis.drive_curve = 0;
      }
    }

    if (fabs(chassis.drive_curve) >= fabs(chassis.drive_sp)) {
      chassis.drive_curve = chassis.drive_sp;
    }

  } else if (fabs(chassis.drive_curve) > fabs(chassis.drive_sp)) {
    float previous_drive_curve = chassis.drive_curve;
    if (chassis.drive_curve > 0) {
      chassis.drive_curve -= deccl_x;
    } else {
      chassis.drive_curve += deccl_x;
    }

    if (fabs(chassis.drive_sp) < 0.003) {
      if (((previous_drive_curve > 0) & (chassis.drive_curve < 0)) |
          ((previous_drive_curve < 0) & (chassis.drive_curve > 0))) {
        chassis.drive_curve = 0;
      }
    } else if (fabs(chassis.drive_curve) < fabs(chassis.drive_sp)) {
      chassis.drive_curve = chassis.drive_sp;
    } else {
      chassis.drive_curve = chassis.drive_sp;
    }
  } else {
    chassis.drive_curve = chassis.drive_sp;
  }
  // }

  chassis._motors[FRONT_RIGHT].speed_sp =
      (chassis.strafe_curve - chassis.drive_curve +
       chassis.rotate_sp * rotate_ratio_fr) *
      wheel_rpm_ratio; // CAN ID: 0x201
  chassis._motors[BACK_RIGHT].speed_sp =
      (-1 * chassis.strafe_curve - chassis.drive_curve +
       chassis.rotate_sp * rotate_ratio_br) *
      wheel_rpm_ratio; // CAN ID: 0x202
  chassis._motors[FRONT_LEFT].speed_sp =
      (chassis.strafe_curve + chassis.drive_curve +
       chassis.rotate_sp * rotate_ratio_fl) *
      wheel_rpm_ratio; // CAN ID: 0x203
  chassis._motors[BACK_LEFT].speed_sp =
      (-1 * chassis.strafe_curve + chassis.drive_curve +
       chassis.rotate_sp * rotate_ratio_bl) *
      wheel_rpm_ratio; // CAN ID: 0x204

  float max = 0.0f;
  // find max item
  int i;
  for (i = 0; i < 4; i++) {
    if (fabsf(chassis._motors[i].speed_sp) > max) {
      max = fabsf(chassis._motors[i].speed_sp);
    }
  }
  // equal proportion
  if (max > MAX_WHEEL_RPM) {
    float rate = MAX_WHEEL_RPM / max;
    int i;
    for (i = 0; i < 4; i++) {
      chassis._motors[i].speed_sp *= rate;
    }
  }

  // speed_limit_handle();
  // Need more consideration!!!s

  for (i = 0; i < 4; i++) {
    if (fabs(chassis._motors[i].speed_curve) <
            fabs(chassis._motors[i].speed_sp) &&
        chassis.ctrl_mode != DODGE_MODE &&
        chassis.ctrl_mode != DODGE_MOVE_MODE) {
      if (chassis._motors[i].speed_sp > 0) {
        chassis._motors[i].speed_curve += accl_value;
      } else {
        chassis._motors[i].speed_curve -= accl_value;
      }

      if (fabs(chassis._motors[i].speed_curve) >
          fabs(chassis._motors[i].speed_sp)) {
        chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
      }
    } else {
      chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
    }
  }

  for (i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    chassis.current[i] =
        chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);
    VAL_LIMIT(chassis.current[i], -16384, 16384);
    // chassis.current[i] = 0;
  }
}

void drive_motor() {
  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
                      chassis.current[FRONT_RIGHT], chassis.current[FRONT_LEFT],
                      chassis.current[BACK_LEFT],
                      chassis.current[BACK_RIGHT]); // BR,FR,--,--
}

uint32_t twist_count;
void chassis_twist_handle() {
  int16_t twist_period = TWIST_PERIOD;
  int16_t twist_angle = TWIST_ANGLE;
  twist_count++;
  chassis.position_ref =
      gimbal_initP +
      twist_angle * sin(2 * M_PI / twist_period * twist_count) * M_PI / 180;
  chassis.rotate_sp = chassis_heading_control(
      &dancing_controller, gimbal_p[0].radian_angle, chassis.position_ref);
}
void dodge_move_handle() {
  int16_t twist_period = TWIST_MOVE_PERIOD;
  int16_t twist_angle = TWIST_MOVE_ANGLE;
  twist_count++;
  chassis.position_ref =
      gimbal_initP +
      twist_angle * sin(2 * M_PI / twist_period * twist_count) * M_PI / 180;
  chassis.rotate_sp = chassis_heading_control(
      &dancing_controller, gimbal_p[0].radian_angle, chassis.position_ref);
  float vy = km.vy;
  float vx = km.vx;
  float angle = (gimbal_p[0].radian_angle - gimbal_initP) * (2.0 / 3.0);
  if (angle > 0) {
    chassis.drive_sp = 0.8 * (vy * cos(angle) + vx * sin(angle));
    chassis.strafe_sp = 0.8 * ((-1) * vy * sin(angle) + vx * cos(angle));
  } else {
    chassis.drive_sp = 0.8 * (vy * cos(-angle) + (-1) * vx * sin(-angle));
    chassis.strafe_sp = 0.8 * (vy * sin(-angle) + vx * cos(-angle));
  }
}
void follow_gimbal_handle() {
  if (twist_count != 0) {
    twist_count = 0;
  }

  float vy = km.vy + rm.vy;
  float vx = km.vx + rm.vx;
  float angle = (gimbal_p[0].radian_angle - gimbal_initP) * (2.0 / 3.0);
  if (angle > 0) {
    chassis.drive_sp = (vy * cos(angle) + vx * sin(angle));
    chassis.strafe_sp = (-1) * vy * sin(angle) + vx * cos(angle);
  } else {
    chassis.drive_sp = (vy * cos(-angle) + (-1) * vx * sin(-angle));
    chassis.strafe_sp = vy * sin(-angle) + vx * cos(-angle);
  }
  chassis.rotate_sp = chassis_heading_control(
      &chassis_heading_controller, gimbal_p[0].radian_angle, gimbal_initP);
}

void chassis_stop_handle() {
  chassis.strafe_sp = 0;
  chassis.drive_sp = 0;
  chassis.rotate_sp = 0;
}

/*
static void chassis_operation_func(int16_t left_right, int16_t front_back,
int16_t rotate)
{

  rc.vx =  left_right / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
  rc.vy =    front_back / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;
  rc.vw =  rotate / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_R;
}
 */
void separate_gimbal_handle() {
  chassis.drive_sp = rm.vy + km.vy;
  chassis.strafe_sp = rm.vx + km.vx;
  chassis.rotate_sp = rm.vw;
}

float chassis_heading_control(pid_controller_t *controller, float get,
                              float set) {
  controller->error[0] = set - get;
  float output =
      controller->error[0] * controller->kp +
      controller->kd * (controller->error[0] - 2 * controller->error[1] +
                        controller->error[2]);
  controller->error[1] = controller->error[0];
  controller->error[2] = controller->error[1];
  return boundOutput(output,100);
}
float power_limit_control(pid_controller_t *controller, float get, float set) {
  controller->error[0] = set - get;
  float output =
      controller->error[0] * controller->kp +
      controller->kd * (controller->error[0] - 2 * controller->error[1] +
                        controller->error[2]);
  controller->error[1] = controller->error[0];
  controller->error[2] = controller->error[1];
  return output;
}

void speed_limit_handle() {
  if (JudgeP->powerInfo.power > 80) {
    if (JudgeP->powerInfo.powerBuffer <= 40) {
      int i;
      for (i = 0; i < 4; i++) {
        if (chassis._motors[i].speed_sp >= chassis._motors[i]._speed &&
            fabs(chassis._motors[i].speed_sp) >=
                fabs(chassis._motors[i]._speed)) {
          chassis._motors[i].speed_curve =
              chassis._motors[i]._speed + accl_value;
          if (chassis._motors[i].speed_curve > chassis._motors[i].speed_sp) {
            chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
          }
        } else if (chassis._motors[i].speed_sp <= chassis._motors[i]._speed &&
                   fabs(chassis._motors[i].speed_sp) >=
                       fabs(chassis._motors[i]._speed)) {
          chassis._motors[i].speed_curve =
              chassis._motors[i]._speed - accl_value;
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
            fabs(chassis._motors[i].speed_sp) >
                fabs(chassis._motors[i].speed_curve)) {
          chassis._motors[i].speed_curve += accl_value;
        } else if (chassis._motors[i].speed_sp <
                       chassis._motors[i].speed_curve &&
                   fabs(chassis._motors[i].speed_sp) >
                       fabs(chassis._motors[i].speed_curve)) {
          chassis._motors[i].speed_curve -= accl_value;
        } else {
          chassis._motors[i].speed_curve = chassis._motors[i].speed_sp;
        }

        if (fabs(chassis._motors[i].speed_curve) >=
            fabs(chassis._motors[i].speed_sp)) {
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
void save_life() {
  chassis.strafe_curve = 0;
  chassis.drive_curve = 0;
  chassis.rotate_sp = 0;
  chassis.strafe_sp = 0;
  chassis.drive_sp = 0;

  if (JudgeP->powerInfo.powerBuffer >= 50) {
    chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
  }
}
void power_limit_handle() {
  if (chassis.strafe_sp > 0) {
    chassis.strafe_curve =
        chassis.strafe_curve - power_limit_control(&power_limit_controller,
                                                   JudgeP->powerInfo.power,
                                                   powerlimit);

    if (chassis.strafe_curve > chassis.strafe_sp) {
      chassis.strafe_curve = chassis.strafe_sp;
    }

    if (chassis.strafe_curve <= 0) {
      chassis.strafe_curve = 0;
    }
  } else if (chassis.strafe_sp < 0) {
    chassis.strafe_curve =
        chassis.strafe_curve + power_limit_control(&power_limit_controller,
                                                   JudgeP->powerInfo.power,
                                                   powerlimit);
    if (chassis.strafe_curve < chassis.strafe_sp) {
      chassis.strafe_curve = chassis.strafe_sp;
    }
    if (chassis.strafe_curve >= 0) {
      chassis.strafe_curve = 0;
    }
  } else {
    chassis.strafe_curve = 0;
  }

  if (chassis.drive_sp > 0) {
    chassis.drive_curve =
        chassis.drive_curve - power_limit_control(&power_limit_controller,
                                                  JudgeP->powerInfo.power,
                                                  powerlimit);

    if (chassis.drive_curve > chassis.drive_sp) {
      chassis.drive_curve = chassis.drive_sp;
    }
    if (chassis.drive_curve <= 0) {
      chassis.drive_curve = 0;
    }
  } else if (chassis.drive_sp < 0) {
    chassis.drive_curve =
        chassis.drive_curve + power_limit_control(&power_limit_controller,
                                                  JudgeP->powerInfo.power,
                                                  powerlimit);
    if (chassis.drive_curve < chassis.drive_sp) {
      chassis.drive_curve = chassis.drive_sp;
    }
    if (chassis.drive_curve >= 0) {
      chassis.drive_curve = 0;
    }
  } else {
    chassis.drive_curve = 0;
  }
  /*
  int power_limit = 80 + 0.5*JudgeP->powerInfo.powerBuffer;
  float a0 = fabs(chassis._encoders[0].act_current * 20.0/16384.0);
  float a1 = fabs(chassis._encoders[1].act_current * 20.0/16384.0);
  float a2 = fabs(chassis._encoders[2].act_current * 20.0/16384.0);
  float a3 = fabs(chassis._encoders[3].act_current * 20.0/16384.0);
  int MAX = (power_limit - 3 - 0.14*(a0*a0+a1*a1+a2*a2+a3*a3))/0.005;


  float A0 = fabs(chassis.current[0] * 20.0/16384.0);
  float A1 = fabs(chassis.current[1]* 20.0/16384.0);
  float A2 = fabs(chassis.current[2] * 20.0/16384.0);
  float A3 = fabs(chassis.current[3]* 20.0/16384.0);
  int SUM =  fabs(chassis._encoders[0].raw_speed)*A0
             +fabs(chassis._encoders[1].raw_speed)*A1
             +fabs(chassis._encoders[2].raw_speed)*A2
             +fabs(chassis._encoders[3].raw_speed)*A3;

  if(SUM > MAX && JudgeP->powerInfo.power >= 80){
    chassis.current[FRONT_RIGHT] = chassis.current[FRONT_RIGHT] * MAX / SUM;
    chassis.current[FRONT_LEFT] = chassis.current[FRONT_LEFT] * MAX / SUM;
    chassis.current[BACK_LEFT] = chassis.current[BACK_LEFT] * MAX / SUM;
    chassis.current[BACK_RIGHT] = chassis.current[BACK_RIGHT] * MAX / SUM;
    chassis.over_power = true;
  }
  else{
    chassis.over_power = false;
  }
*/
}
