/*
 * error.h
 *
 *  Created on: 22 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

#include "ch.h"
#include "hal.h"
#include "main.h"
#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "chassis.h"
#include "shell.h"
#include <string.h>

typedef struct{
  dbus_error_t dbus_error;
  imu_att_error_t imu_error;
  gimbal_error_t gimbal_error;
  ist8310_error_flag_t ist8310_error;
}ERRORStruct, *pEStruct;

pEStruct get_EMsg(void);

void EMsg_init(void);

void EMsg_update(void);
/*
 * Gimbal Error List
 * gimbal_error_t;
 *
 * GIMBAL_YAW_NOT_CONNECTED = 1<<0,
 * GIMBAL_PITCH_NOT_CONNECTED = 1<<1,
 * GIMBAL_INITALIZATION_TIMEOUT = 1<<2,
 * GIMBAL_CONTROL_LOSE_FRAME = 1<<31
 */

//Gimbal Error Messages
#define GIMBAL_ERROR_COUNT    3U
#define GIMBAL_WARNING_COUNT  1U
//static const char gimbal_error_messages[GIMBAL_ERROR_COUNT][50] =
//{
//  "E:Gimbal yaw not connected",
//  "E:Gimbal pitch not connected",
//  "E:Gimbal init timeout"
//};
//
//static const char gimbal_warning_messages[GIMBAL_WARNING_COUNT][50] =
//{
//  "W:Gimbal control lose frame"
//};

void printError_Gimbal(BaseSequentialStream * chp, gimbal_error_t error_Index);

/*
 * IST8310 Error List
 * ist8310_error_flag_t
 *
 * IST8310_INVALID_SAMPLE_RATE = 1
 */

void printError_Ist8310(BaseSequentialStream * chp, ist8310_error_flag_t error_Index);

/*
 * IMU Error List
 * imu_att_error_t;
 *
 * IMU_OK = 0,
 * IMU_CORRUPTED_Q_DATA = 1<<1,
 * IMU_LOSE_FRAME = 1<<31
 */

void printError_IMU(BaseSequentialStream * chp, imu_att_error_t error_Index);

/*
 * Dbus Error List
 * @status: true = connected
 *          false = not connected
 */

void printError_dbus(BaseSequentialStream * chp, dbus_error_t error_Index);

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


void cmd_error(BaseSequentialStream * chp, int argc, char *argv[]);

void error_init(void);

#endif /* INC_ERROR_H_ */


