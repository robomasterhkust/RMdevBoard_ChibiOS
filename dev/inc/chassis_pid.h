#ifndef _CHASSIS_PID_H_
#define _CHASSIS_PID_H_
#include "canBusProcess.h"
typedef struct{

    int16_t error;
    int32_t error_sum;
    int16_t last_error;
    double Kp;
    double Ki;
    double Kd;

}PIDparameter;


float speedPID(float target_value,int encoder_num,PIDparameter* s);
#endif
