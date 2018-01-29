/*
 * error.c
 *
 *  Created on: 22 Jan, 2018
 *      Author: ASUS
 */
#include "main.h"
#include "error.h"

static ERRORStruct EMsg;

pEStruct get_EMsg(void){
  return &EMsg;
}

void EMsg_init(void){
  EMsg.dbus_error = 0;
  EMsg.imu_error = 0;
  EMsg.gimbal_error = 0;
  EMsg.ist8310_error = 0;
  EMsg.chassis_error = 0;
}

void EMsg_update(void){
//  EMsg.imu_error = imu_get_error();
  EMsg.dbus_error = dbus_getError();
  EMsg.gimbal_error = gimbal_getError();
  EMsg.ist8310_error = ist8310_getError();
  EMsg.chassis_error = chassis_getError();
}

void printError_Gimbal(BaseSequentialStream * chp, gimbal_error_t error_Index){
  //if(error_Index ^ 0) chprintf(chp,"Gimbal_Status: GREEN %s\n","");
  if(error_Index & GIMBAL_YAW_NOT_CONNECTED) chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[0]);
  if(error_Index & GIMBAL_PITCH_NOT_CONNECTED) chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[1]);
  if(error_Index & GIMBAL_INITALIZATION_TIMEOUT) chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[2]);
  if(error_Index & GIMBAL_CONTROL_LOSE_FRAME) chprintf(chp,"Gimbal_Warning: %s\n", gimbal_warning_messages[0]);

  //  switch(error_Index){
//  case 0:
//      chprintf(chp,"Gimbal_Status: GREEN %s\n","");
//      break;
//  case GIMBAL_YAW_NOT_CONNECTED:
//      chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[0]);
//      break;
//  case GIMBAL_PITCH_NOT_CONNECTED:
//        chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[1]);
//        break;
//  case GIMBAL_INITALIZATION_TIMEOUT:
//        chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[2]);
//        break;
//  case GIMBAL_CONTROL_LOSE_FRAME:
//        chprintf(chp,"Gimbal_Warning: %s\n", gimbal_warning_messages[0]);
//        break;
//  default:
//        chprintf(chp,"Gimbal_Error: Unknown Error %s\n","");
//        break;
//  }
}

void printError_Ist8310(BaseSequentialStream * chp, ist8310_error_flag_t error_Index){
  if(error_Index & IST8310_INVALID_SAMPLE_RATE) chprintf(chp,"IST8310_Error: Invalid sample rate %s\n", "");
//  switch(error_Index){
//  case 0:
//      chprintf(chp,"IST8310_Status: GREEN %s\n","");
//      break;
//  case IST8310_INVALID_SAMPLE_RATE:
//      chprintf(chp,"IST8310_Error: Invalid sample rate %s\n", "");
//      break;
//  default:
//        chprintf(chp,"IST8310_Error: Unknown Error %s\n","");
//        break;
//  }
}

void printError_IMU(BaseSequentialStream * chp, imu_att_error_t error_Index){
   if(error_Index & IMU_OK) chprintf(chp,"IMU_Status: GREEN %s\n","");
  if(error_Index & IMU_CORRUPTED_Q_DATA) chprintf(chp,"IMU_Status: GREEN %s\n","");
  if(error_Index & IMU_LOSE_FRAME) chprintf(chp,"IMU_Error: IMU lose frame %s\n", "");

//  switch(error_Index){
//  case IMU_OK:
//      chprintf(chp,"IMU_Status: GREEN %s\n","");
//      break;
//  case IMU_CORRUPTED_Q_DATA:
//      chprintf(chp,"IMU_Error: IMU reading corrupted %s\n", "");
//      break;
//  case IMU_LOSE_FRAME:
//        chprintf(chp,"IMU_Error: IMU lose frame %s\n", "");
//        break;
//  default:
//        chprintf(chp,"IMU_Error: Unknown Error %s\n","");
//        break;
//  }

}

void printError_dbus(BaseSequentialStream * chp, dbus_error_t error_Index){
  switch(error_Index){
  case 0:
      chprintf(chp,"Dbus Status: Not connected %s\n","");
      break;
  case 1:
      chprintf(chp,"Dbus Status: Connected %s\n", "");
      break;
 default:
        chprintf(chp,"Dbus Error: Unknown Error %s\n","");
        break;
  }
}

void printError_chassis(BaseSequentialStream * chp, chassis_error_t error_Index){
  if(error_Index == 0) chprintf(chp,"Chassis Status: GREEN %s\n","");
  if(error_Index & CHASSIS_MOTOR_NOT_CONNECTED) chprintf(chp,"Chassis Error: Motor Not Connected %s\n","");
}

void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{
      (void) argc,argv;
      printError_Gimbal(chp, EMsg.gimbal_error);
      printError_Ist8310(chp, EMsg.ist8310_error);
      printError_IMU(chp, EMsg.imu_error);
      printError_dbus(chp, EMsg.dbus_error);
      printError_chassis(chp, EMsg.chassis_error);
}

static THD_WORKING_AREA(error_message_thread_wa, 512);
static THD_FUNCTION(error_message_thread, p)
{
  (void)p;
  chRegSetThreadName("Error Messages");

  while(!chThdShouldTerminateX())
  {
    EMsg_update();
    chThdSleepMilliseconds(100);
  }
}

void error_init(void){
  EMsg_init();
  chThdCreateStatic(error_message_thread_wa, sizeof(error_message_thread_wa),
                      NORMALPRIO + 7,
                      error_message_thread, NULL);
}
