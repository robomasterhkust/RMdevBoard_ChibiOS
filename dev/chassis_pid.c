
#include "chassis_pid.h"

float speedPID(float target_value,int encoder_num,PIDparameter* s){



                ChassisEncoder_canStruct* encoder_array = can_getChassisMotor();
                ChassisEncoder_canStruct encoder = encoder_array[encoder_num];


                float output;
                s->error = -target_value + encoder.raw_speed;
                s->error_sum += s->error;


                if(s->error_sum > 80000){
                        s->error_sum = 80000;
                }
                if(s->error_sum < -80000)
                {
                        s->error_sum = -80000;
                }

                output = s->Kp*s->error + s->Ki*s->error_sum + s->Kd*(s->error - s->last_error);
                return output;
}



