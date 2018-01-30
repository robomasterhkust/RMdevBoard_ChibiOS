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

// DBUS MACRO

#define CURRENT_MAX    ((int16_t) 277)              //
#define CURRENT_MIN    ((int16_t)-277)              //
#define CHASSIS_GEAR_RATIO    27U

#define RPM_MAX    ((int16_t) 277)              //
#define RPM_MIN    ((int16_t) -277)              //

#define HEADING_MIN     ((float) -3.14159) // - pi
#define HEADING_MAX     ((float) 3.14159)   // pi
//#define HEADING_MIN     ((float) -1) // - pi
//#define HEADING_MAX     ((float) 1)   // pi
#define HEADING_SCALE   ((uint16_t) 1)

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x

//#define CHASSIS_USE_POS_MOTOR

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
  float pid_last_error;

  ChassisEncoder_canStruct* _encoders;
  PGyroStruct _pGyro;
} chassisStruct;

// MATH definition

chassis_error_t chassis_getError(void);
chassisStruct* chassis_get(void);
void chassis_init(void);
void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1);
#endif /* INC_CHASSIS_H_ */
