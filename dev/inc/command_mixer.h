//
// Created by beck on 29/10/18.
//

#ifndef RMDEVBOARD_CHIBIOS_COMMAND_MIXER_H
#define RMDEVBOARD_CHIBIOS_COMMAND_MIXER_H

#include "dbus.h"
#include "keyboard.h"
#include "canBusProcess.h"
#include "chassis_config.h"

void command_mixer_init();

typedef enum control_mode_e
{
    CTL_CHASSIS_RELAX          = 0,
    CTL_CHASSIS_STOP           = 1,
    CTL_MANUAL_SEPARATE_GIMBAL = 2,
    CTL_MANUAL_FOLLOW_GIMBAL   = 3,
    CTL_DODGE_MODE             = 4,
    CTL_AUTO_SEPARATE_GIMBAL   = 5,
    CTL_AUTO_FOLLOW_GIMBAL     = 6,
    CTL_DODGE_MOVE_MODE        = 7,
    CTL_SAVE_LIFE              = 8,
} control_mode_e;

#define CMD_MIXER_UPDATE_FREQ 100
#define CMD_MIXER_UPDATE_PERIOD_US (1000000 / CMD_MIXER_UPDATE_FREQ)


typedef struct
{
    float           vx;
    float           vy;
    float           vw;

    control_mode_e  ctrl_mode;
    control_mode_e  last_ctrl_mode;

    bool            reboot_flag;

} command_t;

command_t* cmd_mixer_get(void);

#endif //RMDEVBOARD_CHIBIOS_COMMAND_MIXER_H
