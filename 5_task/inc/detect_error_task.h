//
// Created by beck on 2/4/2018.
//

#ifndef RM_CHIBIOS_DETECT_ERROR_TASK_H
#define RM_CHIBIOS_DETECT_ERROR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

bool is_motor_power_on(void);
bool power_failure(void);
void detect_error_task_init(void);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_DETECT_ERROR_TASK_H
