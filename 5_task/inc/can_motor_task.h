//
// Created by beck on 5/4/2018.
//

#ifndef RM_CHIBIOS_CAN_MOTOR_TASK_H
#define RM_CHIBIOS_CAN_MOTOR_TASK_H

#include "can_bus.h"
#include "stdint.h"
#include "stdbool.h"

#define CAN_ENCODER_RANGE           8192            // 0x2000
#define CAN_ENCODER_RADIAN_RATIO    7.669904e-4f    // 2*M_PI / 0x2000

#define GIMBAL_MOTOR_NUM  2U
#define CHASSIS_MOTOR_NUM 4U
#define EXTRA_MOTOR_NUM   4U

typedef enum
{
    GIMBAL_YAW = 0,
    GIMBAL_PITCH
} gimbal_num_t;

typedef enum
{
    FRONT_LEFT = 1,
    FRONT_RIGHT = 0,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
} chassis_num_t;

typedef struct
{
    uint16_t raw_angle;
    int16_t raw_current;
    int16_t current_setpoint;

    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} GimbalEncoder_canStruct;

typedef struct
{
    uint16_t raw_angle;
    int16_t raw_speed;
    int16_t act_current;
    uint8_t temperature;

    uint16_t last_raw_angle;
    uint16_t offset_raw_angle;
    uint32_t msg_count;
    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} ChassisEncoder_canStruct;

#ifdef __cplusplus
extern "C" {
#endif

volatile GimbalEncoder_canStruct *can_getGimbalMotor(void);

volatile ChassisEncoder_canStruct *can_getChassisMotor(void);

volatile ChassisEncoder_canStruct *can_getExtraMotor(void);

void can_process_chassis_encoder(const CANRxFrame * rxmsg);

void can_process_extra_encoder(const CANRxFrame * rxmsg);

void can_process_gimbal_encoder(const CANRxFrame * rxmsg);

void can_motorSetCurrent(CANDriver * CANx,
                         uint16_t EID,
                         int16_t cm1_iq,
                         int16_t cm2_iq,
                         int16_t cm3_iq,
                         int16_t cm4_iq);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_CAN_MOTOR_TASK_H
