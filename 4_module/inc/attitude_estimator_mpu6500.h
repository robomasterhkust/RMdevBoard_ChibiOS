//
// Created by beck on 2/4/2018.
//

#ifndef RM_CHIBIOS_ATTITUDE_MPU6500_H
#define RM_CHIBIOS_ATTITUDE_MPU6500_H

#include "stdint.h"
#include "mpu6500.h"

#define ATT_W_ACCEL_6500     0.3f
#define ATT_W_GYRO_6500      0.1f
#define GYRO_BIAS_MAX_6500  0.05f

#ifdef __cplusplus
extern "C" {
#endif

uint8_t attitude_estimator_mpu6500_init(PIMUStruct pIMU);
uint8_t attitude_estimator_mpu6500_update(PIMUStruct pIMU);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_ATTITUDE_MPU6500_H
