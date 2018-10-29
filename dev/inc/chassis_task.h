//
// Created by beck on 29/10/18.
//

#ifndef RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H
#define RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H

#include "canBusProcess.h"
#include "chassis_config.h"
#include "chassis.h"

// chassis control period frequency
#define CHASSIS_UPDATE_FREQ 100
#define CHASSIS_UPDATE_PERIOD_US 1000000/CHASSIS_UPDATE_FREQ

#define CHASSIS_CAN_PORT    &CAND1
#define CHASSIS_CAN_SID     0x200

#define CHASSIS_CONNECTION_ERROR_COUNT 10U

typedef uint8_t chassis_error_t;

/*
typedef enum chassis_mode_e
{
    CHASSIS_RELAX          = 0,
    CHASSIS_STOP           = 1,
    MANUAL_SEPARATE_GIMBAL = 2,
    MANUAL_FOLLOW_GIMBAL   = 3,
    DODGE_MODE             = 4,
    AUTO_SEPARATE_GIMBAL   = 5,
    AUTO_FOLLOW_GIMBAL     = 6,
    DODGE_MOVE_MODE        = 7,
    SAVE_LIFE              = 8,
} chassis_mode_e;
 */


typedef struct
{
    float   speed_sp;
    float   _speed;
    uint8_t _wait_count;
} chassis_motor_t;

typedef struct
{
    chassis_motor_t _motors[CHASSIS_MOTOR_NUM];

    float           heading_sp;
    float           rotation_sp;
    float           drive_sp;
    float           straft_sp;
    int16_t         rotate_x_offset;
    int16_t         rotate_y_offset;

    chassis_mode_e  ctrl_mode;
    chassis_mode_e  last_ctrl_mode;

    float           power_limit;

    int16_t         current[4];

    uint8_t         error_flag;
} chassis_t;

void chassis_task_init();

volatile chassis_t *chassis_struct_get(void);

#endif //RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H
