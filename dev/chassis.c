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


int16_t FR;
int16_t BR;
int16_t FL;
int16_t BL;

volatile int16_t   front_right ; // CAN ID: 0x201
volatile int16_t   back_right  ;  // CAN ID: 0x202
volatile int16_t   front_left  ;  // CAN ID: 0x203
volatile int16_t   back_left   ; // CAN ID: 0x204

extern PID CM1PID;
extern PID CM2PID;
extern PID CM3PID;
extern PID CM4PID;

volatile int16_t drive;
volatile int16_t strafe ;
volatile int16_t rotate ;

// MATH function
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




//*******************************************************

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
    (void)p;
    chRegSetThreadName("chassis controller");

    RC_Ctl_t* pRC = RC_get();

    while(1){

      drive_kinematics(pRC->rc.channel0, pRC->rc.channel1, pRC->rc.channel2);
    //  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
          		10000, 10000, 10000, 10000);
      chThdSleepMilliseconds(10);
    }
}

void chassis_init(void){

  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);
}



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
    strafe  = (int16_t)map(RX_X2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    drive = (int16_t)map(RX_Y1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);
    rotate = (int16_t)map(RX_X1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, RPM_MIN, RPM_MAX);

    // For later coordinate with Gimbal
    int rotate_feedback = 0;

    front_right = ((strafe - drive + rotate) + rotate_feedback);   // CAN ID: 0x201
    back_right = ((-1*strafe - drive + rotate) + rotate_feedback);       // CAN ID: 0x202
    front_left = ((strafe + drive + rotate) + rotate_feedback);       // CAN ID: 0x203
    back_left = ((-1*strafe + drive + rotate) + rotate_feedback);     // CAN ID: 0x204

    // Set a scaling factor to allow different speed modes, controlled by the left switch
    // POSITION(value) = mode: UP(1) = fast, MIDDLE(3) = normal, DOWN(2) = slow

    //Update using CAN bus chassis function


    FR = PID_output(0, front_right, &CM1PID);
    BR = PID_output(1, back_right , &CM2PID);
    FL = PID_output(2, front_left , &CM3PID);
    BL = PID_output(3, back_left  , &CM4PID);


    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    		FR, BR, FL, BL);

  //  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    		front_right * 0.1 , back_right * 0.1, front_left * 0.1 , back_left * 0.1);

}
