/*
 * chassis.h
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

#define CHASSIS_UPDATE_FREQ 500
#define CHASSIS_UPDATE_PERIOD_US 1000000/CHASSIS_UPDATE_FREQ

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\


/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS     76
/* the perimeter of wheel(mm) */
#define PERIMETER  478

/* wheel track distance(mm) */
#define WHEELTRACK  545 //403
/* wheelbase distance(mm) */
#define WHEELBASE  340 //385

/* gimbal is relative to chassis center x axis offset(mm) */
#define GIMBAL_X_OFFSET 120 //150
/* gimbal is relative to chassis center y axis offset(mm) */
#define GIMBAL_Y_OFFSET 0

/* chassis motor use 3508 default */
/* define CHASSIS_EC60 to use EC60 */

  /* chassis motor use 3508 */
  /* the deceleration ratio of chassis motor */
//  #define CHASSIS_DECELE_RATIO (1.0f/27.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        310 //8000  //8347rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300   //5000rpm

//Codes above are copied from Official


// DBUS MACRO
#define CHASSIS_GEAR_RATIO    27U
//#define RPM_MAX    ((int16_t) 350)              //
//#define RPM_MIN    ((int16_t) -350)              //
#define HEADING_MIN     ((float) -3.14159) // - pi
#define HEADING_MAX     ((float) 3.14159)   // pi
#define HEADING_SCALE   ((uint16_t) 1)
#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x
#define RADIAN_COEF 57.3f
//#define CHASSIS_USE_POS_MOTOR

typedef enum
{
  CHASSIS_RELAX = 0,
  CHASSIS_STOP = 1,
  MANUAL_SEPARATE_GIMBAL =2,
  MANUAL_FOLLOW_GIMBAL =3,
  DODGE_MODE = 4,
  AUTO_SEPARATE_GIMBAL =5,
  AUTO_FOLLOW_GIMBAL =6,
}chassis_mode_e;

typedef struct{
  float speed_sp;
  float _speed;
  uint8_t _wait_count;
} motorStruct;

typedef struct{
  float speed_sp;
  float _speed;
  float pos_sp;
  float _pos;
  uint8_t _wait_count;
} motorPosStruct;

typedef enum {
  CHASSIS_MOTOR_0_NOT_CONNECTED = 1 << 0,
  CHASSIS_MOTOR_1_NOT_CONNECTED = 1 << 1,
  CHASSIS_MOTOR_2_NOT_CONNECTED = 1 << 2,
  CHASSIS_MOTOR_3_NOT_CONNECTED = 1 << 3
};

typedef enum {
  CHASSIS_OK = 0,
  CHASSIS_MOTOR_NOT_CONNECTED = 1 <<0
};
typedef uint8_t chassis_error_t;

typedef struct{
  motorStruct _motors[CHASSIS_MOTOR_NUM];

  #ifdef CHASSIS_USE_POS_MOTOR
    motorPosStruct pos_motors[4];
  #endif

  float heading_sp;
  float rotate_sp;
  float drive_sp;
  float strafe_sp;
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  int16_t         current[4];

//  int16_t       position_ref;
//  uint8_t       follow_gimbal;


  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;

  float pid_last_error;
  uint8_t errorFlag;

  ChassisEncoder_canStruct* _encoders;
  PGyroStruct _pGyro;
} chassisStruct;

// MATH definition
/*
 * define by official group
 *
 *
 *
 * */
void mecanum_calc(void);



/*define by UST*/
chassis_error_t chassis_getError(void);
chassisStruct* chassis_get(void);
void chassis_init(void);
void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1);
void drive_motor(void);

void chassis_twist_handle(void);
void chassis_stop_handle(void);
void separate_gimbal_handle(int RX_X2, int RX_Y1, int RX_X1);
void follow_gimbal_handle(void);







#endif /* INC_CHASSIS_H_ */
