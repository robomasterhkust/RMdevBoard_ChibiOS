/*
 * error.h
 *
 *  Created on: 22 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

typedef struct{
  imu_att_error_t imu_error;
  gimbal_error_t gimbal_error;
  ist8310_error_flag_t ist8310_error;

}ERRORStruct, *pEStruct;

static ERRORStruct EMsg;

pEStruct get_EMsg(void){
  return &EMsg;
}

void EMsg_init(void){
  EMsg.imu_error = 0;
  EMsg.gimbal_error = 0;
  EMsg.ist8310_error = 0;
}

void EMsg_update(void){
//  EMsg.imu_error = imu_get_error();
  EMsg.gimbal_error = gimbal_get_error();
  EMsg.ist8310_error = ist8310_get_error();
}

//Gimbal Error List

//typedef enum {
//  GIMBAL_YAW_NOT_CONNECTED = 1<<0,
//  GIMBAL_PITCH_NOT_CONNECTED = 1<<1,
//  GIMBAL_INITALIZATION_TIMEOUT = 1<<2,
//  GIMBAL_CONTROL_LOSE_FRAME = 1<<31
//} gimbal_error_t;

//Gimbal Error Messages
#define GIMBAL_ERROR_COUNT    3U
#define GIMBAL_WARNING_COUNT  1U
static const char gimbal_error_messages[][GIMBAL_ERROR_COUNT] =
{
  "E:Gimbal yaw not connected",
  "E:Gimbal pitch not connected",
  "E:Gimbal init timeout"
};

static const char gimbal_warning_messages[][GIMBAL_WARNING_COUNT] =
{
  "W:Gimbal control lose frame"
};

void printError_Gimbal(BaseSequentialStream * chp, gimbal_error_t* error_Index){
  switch(*error_Index){
  case 0:
      chprintf(chp,"Gimbal_Status: GREEN %s\n","");
  case 1<<0:
      chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[0]);
  case 1<<1:
        chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[1]);
  case 1<<2:
        chprintf(chp,"Gimbal_Error: %s\n", gimbal_error_messages[2]);
  case 1<<31:
        chprintf(chp,"Gimbal_Warning: %s\n", gimbal_warning_messages[0]);
  default:
        chprintf(chp,"Gimbal_Error: Unknown Error %s\n","");
  }
}

// IST8310 Error List

//typedef enum{
//  IST8310_INVALID_SAMPLE_RATE = 1
//} ist8310_error_flag_t;

void printError_Ist8310(BaseSequentialStream * chp, ist8310_error_flag_t* error_Index){
  switch(*error_Index){
  case 0:
      chprintf(chp,"IST8310_Status: GREEN %s\n","");
  case 1<<0:
      chprintf(chp,"IST8310_Error: Invalid sample rate %s\n", "");
  default:
        chprintf(chp,"IST8310_Error: Unknown Error %s\n","");
  }
}

// IMU Error List

//typedef enum {
//  IMU_OK = 0,
//  IMU_CORRUPTED_Q_DATA = 1<<1,
//  IMU_LOSE_FRAME = 1<<31
//} imu_att_error_t;
//

void printError_IMU(BaseSequentialStream * chp, gimbal_error_t* error_Index){
  switch(*error_Index){
  case 0:
      chprintf(chp,"IMU_Status: GREEN %s\n","");
  case 1<<1:
      chprintf(chp,"IMU_Error: Invalid sample rate %s\n", "");
  default:
        chprintf(chp,"IMU_Error: Unknown Error %s\n","");
  }
}


//
///* Error codes */
//
//#define CH_FAILED 0
//#define CH_SUCCESS 1
///** @brief Flash operation successful */
//#define FLASH_RETURN_SUCCESS CH_SUCCESS
//
///** @brief Flash operation error because of denied access, corrupted memory.*/
//#define FLASH_RETURN_NO_PERMISSION -1
//
///** @brief Flash operation error because of bad flash, corrupted memory */
//#define FLASH_RETURN_BAD_FLASH -11
//


void cmd_error(BaseSequentialStream * chp, int argc, char *argv[])
{
      (void) argc,argv;

}

#endif /* INC_ERROR_H_ */
