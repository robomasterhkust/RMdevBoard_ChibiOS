/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */
#include "chassis.h"
#include "chassis_pid.h"
#define chassis_canUpdate()   \
  (can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    0, 0, 0, 0))

// Variables
volatile int32_t front_right = 0;
volatile int32_t back_right = 0;
volatile int32_t front_left = 0;
volatile int32_t back_left=0;
float input_front_right =0;
float input_back_right =0;
float input_front_left=0;
float input_back_left=0;

PIDparameter FR_wheel={0,0,0,0,0,0};
PIDparameter FL_wheel={0,0,0,0,0,0};
PIDparameter BR_wheel={0,0,0,0,0,0};
PIDparameter BL_wheel={0,0,0,0,0,0};
// MATH function
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Debugging unit**************************
static uint16_t drive;

uint16_t* get_drive(void){
  return &drive;
}

void drive_init(void){
  drive = 0;
}
//*********************************************

// function ported from INTERNAL: needs further testing
void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1)
{
    // Set dead-zone to 6% range to provide smoother control
    float THRESHOLD = (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN)*3/100;

    // Create "dead-zone" for drive
    if(ABS(RX_X2 - RC_CH_VALUE_OFFSET) < THRESHOLD)  RX_X2 = RC_CH_VALUE_OFFSET;

    // Create "dead-zone" for strafe
    if(ABS(RX_Y1 - RC_CH_VALUE_OFFSET) < THRESHOLD)  RX_Y1 = RC_CH_VALUE_OFFSET;

    // Create "dead-zone" for rotate
    if(ABS(RX_X1 - RC_CH_VALUE_OFFSET) < THRESHOLD) RX_X1 = RC_CH_VALUE_OFFSET;

    //Remote Control Commands, Mapped to match min and max RPM
    int16_t drive  = (int16_t)map(RX_X2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    int16_t strafe = (int16_t)map(RX_Y1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    int16_t rotate = -(int16_t)map(RX_X1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);

    // For later coordinate with Gimbal
    int rotate_feedback = 0;

    front_right = (-1*drive + strafe + rotate) + rotate_feedback;   // CAN ID: 0x201
    back_right = (drive + strafe + rotate) + rotate_feedback;       // CAN ID: 0x202
    front_left = (drive - strafe + rotate) + rotate_feedback;       // CAN ID: 0x203
    back_left = (-1*drive - strafe + rotate) + rotate_feedback;     // CAN ID: 0x204

    input_front_right = speedPID(front_right,0,&FR_wheel);
    input_back_right = speedPID(back_right,1,&BR_wheel);
    input_back_left = speedPID(back_left,2,&BL_wheel);
    input_front_left = speedPID(front_left,3,&FL_wheel);

    //Update using CAN bus chassis function (provided by Alex Wong)
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,input_front_right, input_back_right, input_front_left, input_back_left);

}

//*******************************************************

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
    (void)p;
    chRegSetThreadName("chassis controller");

    RC_Ctl_t* pRC = RC_get();

    while(1){

      drive_kinematics(pRC -> rc.channel0,pRC -> rc.channel1,pRC -> rc.channel2);
      chThdSleepMilliseconds(10);
    }
}

void chassis_init(void){
  drive_init();
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);
}

