//
// Created by beck on 29/10/18.
//

#ifndef RMDEVBOARD_CHIBIOS_CHASSIS_CONFIG_H
#define RMDEVBOARD_CHIBIOS_CHASSIS_CONFIG_H

// the radius of wheel, mm
#define MECANUM_WHEEL_RADIUS     76
// the perimeter of wheel, mm
#define MECANUM_WHEEL_PERIMETER  478

// wheel track distancem, mm
#define WHEELTRACK  420 //403
// wheelbase distance, mm
#define WHEELBASE  307

// gimbal relative to chassis center x axis offset(mm)
#define GIMBAL_X_OFFSET 87
// gimbal relative to chassis center y axis offset(mm)
#define GIMBAL_Y_OFFSET 0

// RM3508 gear ratio
#define MOTOR_GEAR_RATIO    ( 187.0f / 3591.0f )
// RM3508 motor maximum speed, rpm
#define MAX_WHEEL_RPM        8500
// chassis maximum translation speed, mm/s
#define MAX_CHASSIS_VX_SPEED 3300
#define MAX_CHASSIS_VY_SPEED 3300
// chassis maximum rotation speed, degree/s
#define MAX_CHASSIS_VR_SPEED 300

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

#endif //RMDEVBOARD_CHIBIOS_CHASSIS_CONFIG_H
