//
// Created by beck on 2/4/2018.
//

#ifndef RM_CHIBIOS_ATTITUDE_ESTIMATOR_TASK_H
#define RM_CHIBIOS_ATTITUDE_ESTIMATOR_TASK_H

#include "attitude.h"
#include "attitude_estimator_mpu6500.h"

#define IMU_TEMP_SETPOINT 61.0f


#ifdef __cplusplus
extern "C" {
#endif

void attitude_estimator_init(void);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_ATTITUDE_ESTIMATOR_TASK_H
