//
// Created by logan on 10/1/18.
//


#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "adis16265.h"

#include "complementary_filter.h"

#define COMPLEMENTARY_FILTER_CAN &CAND1
#define COMPLEMENTARY_FILTER_EID 0x111

static float output;
/*float get_filter_output(void)
{
    return output;
}*/


void can_send_yaw_diff(CANDriver *const CANx, uint8_t _error) {
    filter_send_struct filter_send_struct1;
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = COMPLEMENTARY_FILTER_EID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();
    if(_error){
        filter_send_struct1.filter_output = 0.0f;
        filter_send_struct1.error = 1;
    }
    else{
        filter_send_struct1.filter_output = output;
        filter_send_struct1.error = 0;
    }
    memcpy(&(txmsg.data8), &filter_send_struct1, 8);
    chSysUnlock();

    canTransmit(CANx, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}

static GimbalStruct* p_gimbal;
static PGyroStruct pGyro;
static volatile GimbalEncoder_canStruct * p_received_gimbal;

static float angular_velocity;
static float encoder_radian_init;
static float encoder_radian;

static float angular_velocity_dt = 1.0f / ((float)Filter_Frequency);

static float coefficient = 0.98f;


static uint8_t filter_should_start;
static uint8_t filter_inited;

static THD_WORKING_AREA(filter_thread_wa, 512);
static THD_FUNCTION(filter_thread, p){

    systime_t tick = chVTGetSystemTimeX();

    while (!chThdShouldTerminateX()) {
        tick += Filter_PERIOD_ST;
        if (tick > chVTGetSystemTimeX())
            chThdSleepUntil(tick);
        else {
            tick = chVTGetSystemTimeX();
        }


        if (pGyro->state == INITED && \
            p_gimbal->state != GIMBAL_STATE_UNINIT && \
            p_gimbal->state != GIMBAL_STATE_INITING && \
            p_gimbal->motor[GIMBAL_YAW]._wait_count <= GIMBAL_CONNECTION_ERROR_COUNT)
        {
            filter_should_start = 1;
            encoder_radian_init = get_yaw_init_pos();
        }
        else{
            filter_should_start = 0;
            filter_inited = 0;
        }


        if(filter_should_start == 1){
            if(filter_inited == 0)
            {
                output = p_received_gimbal[GIMBAL_YAW].radian_angle * 0.533f - encoder_radian_init;
                filter_inited = 1;
              //  can_send_yaw_diff(COMPLEMENTARY_FILTER_CAN, 1);
            }
            else{
                angular_velocity = pGyro->angle_vel;
                encoder_radian = p_received_gimbal[GIMBAL_YAW].radian_angle * 0.533f - encoder_radian_init;
                output = coefficient * (output + angular_velocity * angular_velocity_dt)\
                        + (1.0f - coefficient) * encoder_radian;
                //can_send_yaw_diff(COMPLEMENTARY_FILTER_CAN, 0);
            }
        }
        else{
            //can_send_yaw_diff(COMPLEMENTARY_FILTER_CAN, 1);
            continue;
        }

    }

}

void filter_init(void){

    pGyro = gyro_get();
    p_gimbal = gimbal_get();
    p_received_gimbal = can_getGimbalMotor();

    angular_velocity = 0.0f;
    encoder_radian_init = 0.0f;
    encoder_radian = 0.0f;

    output = 0.0f;

    filter_should_start = 0;
    filter_inited = 0;

    chThdCreateStatic(filter_thread_wa, sizeof(filter_thread_wa),
                      NORMALPRIO, filter_thread, NULL);
}