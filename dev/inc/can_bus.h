//
// Created by beck on 5/4/2018.
//

#ifndef RM_CHIBIOS_CAN_BUS_H
#define RM_CHIBIOS_CAN_BUS_H

#include "can_motor_task.h"
#include "can_communication_task.h"
#include "uwb_driver.h"

#define GIMBAL_MOTOR_NUM    2U
#define CHASSIS_MOTOR_NUM   4U
#define EXTRA_MOTOR_NUM     4U
#define UWB_NUM             1U

#define CAN_FILTER_NUM      28U

/* CAN Bus standard ID for board to board communication */
#define CAN_GIMBAL_BOARD_ID                         0x001
#define CAN_CHASSIS_BOARD_ID                        0x002
#define CAN_CHASSIS_BOARD_GAMEINFO_ID                0x003
#define CAN_CHASSIS_BOARD_PROJECTILE_HLTH_ID        0x004
#define CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID        0x005
#define CAN_CHASSIS_BOARD_VOLT_CURRENT_ID            0x006
#define CAN_CHASSIS_BOARD_SHOOTERHEAT_RFID_BUFFERINFO_ID            0x007
#define CAN_CHASSIS_BOARD_LOCATION_X_Y_ID            0x008
#define CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID            0x009
#define CAN_NVIDIA_TX2_BOARD_ID                     0x103

/* CAN Bus standard ID for motors  */
#define CAN_CHASSIS_FL_FEEDBACK_MSG_ID              0x202
#define CAN_CHASSIS_FR_FEEDBACK_MSG_ID              0x201
#define CAN_CHASSIS_BL_FEEDBACK_MSG_ID              0x203
#define CAN_CHASSIS_BR_FEEDBACK_MSG_ID              0x204
#define CAN_GIMBAL_YAW_FEEDBACK_MSG_ID              0x205
#define CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID            0x206

#define CAN_C620_STD_ID_FEEDBACK_MSG_ID_1           0x201
#define CAN_C620_STD_ID_FEEDBACK_MSG_ID_2           0x202
#define CAN_C620_STD_ID_FEEDBACK_MSG_ID_3           0x203
#define CAN_C620_STD_ID_FEEDBACK_MSG_ID_4           0x204

/* CAN Bus standard ID for UWB module*/
#define CAN_UWB_MSG_ID                              0x259

/* CAN Bus command EID for all motors */
#define CAN_CHASSIS_CAN_EID                         0x200
#define CAN_GIMBAL_CAN_EID                          0x1FF
#define CAN_RM6623_EXTRA_EID                        0x2FF
#define CAN_C620_EXTRA_EID                          0x1FF

/* CAN Bus extra ID for RM3508 motors   */
#define CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_5         0x205
#define CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_6         0x206
#define CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_7         0x207
#define CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_8         0x208

/* CAN Bus extra ID for RM6623 motors   */
#define CAN_GIMBAL_ROLL_FEEDBACK_MSG_ID             0x207
#define CAN_RM6623_RESV_FEEDBACK_MSG_ID             0x208
#define CAN_RM6623_EX1_FEEDBACK_MSG_ID              0x209
#define CAN_RM6623_EX2_FEEDBACK_MSG_ID              0x20A
#define CAN_RM6623_EX3_FEEDBACK_MSG_ID              0x20B
#define CAN_RM6623_EX4_FEEDBACK_MSG_ID              0x20C

#ifdef __cplusplus
extern "C" {
#endif

void can_bus_init(void);

#ifdef __cplusplus
}
#endif


#endif //RM_CHIBIOS_CAN_BUS_H
