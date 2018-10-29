/*
 * Created by beck on 29/10/18.
 * handle chassis state change and
 */
#include <canBusProcess.h>
#include "ch.h"
#include "hal.h"
#include "command_mixer.h"
#include "math_misc.h"

static command_t cmd_mixer;

command_t *
cmd_mixer_get(void)
{ return &cmd_mixer; }


volatile Gimbal_Send_Dbus_canStruct *dbus_msg;
volatile Ros_msg_canStruct *ros_msg;

lpfilterStruct lp_rise_cmd;
lpfilterStruct lp_fall_cmd;
static THD_WORKING_AREA(command_mixer_wa, 512);

static THD_FUNCTION(command_mixer, p)
{
    (void) p;

    dbus_msg = can_get_sent_dbus();
    ros_msg = can_get_ros_msg();

    uint32_t tick = chVTGetSystemTimeX();
    while (!chThdShouldTerminateX()) {
        // real-time control
        tick += US2ST(CMD_MIXER_UPDATE_PERIOD_US);
        if (tick > chVTGetSystemTimeX()) {
            chThdSleepUntil(tick);
        } else {
            tick = chVTGetSystemTimeX();
        }

        // command value pre-process and addition
        double vx_dbus = (dbus_msg->channel0 - CMD_MIXER_RC_MID) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
        double vy_dbus = (dbus_msg->channel1 - CMD_MIXER_RC_MID) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;

        double vx_ros  = ros_msg->py; // mm/s
        double vy_ros  = ros_msg->pz; // mm/s


        // state machine
    }
}

void command_mixer_init()
{
    lpfilter_init(&lp_rise_cmd, CMD_MIXER_UPDATE_FREQ, 5);
    lpfilter_init(&lp_fall_cmd, CMD_MIXER_UPDATE_FREQ, 30);
    chThdCreateStatic(command_mixer_wa, sizeof(command_mixer_wa), NORMALPRIO,
                      command_mixer, NULL);
}