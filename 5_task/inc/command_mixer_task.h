//
// Created by beck on 31/3/2018.
//

#ifndef RM_CHIBIOS_COMMAND_MIXER_H
#define RM_CHIBIOS_COMMAND_MIXER_H

typedef struct {
    float yaw_atti_cmd;
    float yaw_vel_cmd;
    float pitch_atti_cmd;
    float pitch_vel_cmd;
} GimbalCommandStruct;

GimbalCommandStruct* gimbal_cmd_get(void);
void command_mixer_init(void);
#endif //RM_CHIBIOS_COMMAND_MIXER_H
