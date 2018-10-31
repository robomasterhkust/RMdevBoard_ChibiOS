//
// Created by beck on 29/10/18.
//

#ifndef RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H
#define RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H

#include "canBusProcess.h"
#include "chassis_config.h"
#include "chassis.h"

// chassis control period frequency
#define CHASSIS_TASK_UPDATE_FREQ 100
#define CHASSIS_TASK_UPDATE_PERIOD_US (1000000/CHASSIS_TASK_UPDATE_FREQ)

#define CHASSIS_CAN_PORT    &CAND1
#define CHASSIS_CAN_SID     0x200

#define CHASSIS_CONNECTION_ERROR_COUNT 10U

#define MOTOR_OUTPUT_MAX 16384

typedef uint8_t chassis_error_t;


typedef struct
{
    float   speed_sp;
    float   _speed;
    float   _speed_raw;
    uint8_t _wait_count;
} chassis_motor_t;

typedef struct
{
    chassis_motor_t     _motors[CHASSIS_MOTOR_NUM];
    pid_controller_t    motor_vel_ctrl[CHASSIS_MOTOR_NUM];
    pid_controller_t    heading_ctrl;

    float               vx_sp;
    float               vy_sp;
    float               vw_sp;

    int16_t             rotate_x_offset;
    int16_t             rotate_y_offset;
    float               position_ref;

    int16_t             current[4];

    uint8_t             error_flag;
} chassis_t;

void chassis_task_init();

volatile chassis_t *chassis_struct_get(void);

#endif //RMDEVBOARD_CHIBIOS_CHASSIS_TASK_H
