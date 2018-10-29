/*
 * Created by beck on 29/10/18.
 * handle chassis state change and
 */
#include <canBusProcess.h>
#include <math_misc.h>
#include <command_mixer.h>
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

lpfilterStruct lp_rise_cmd_x;
lpfilterStruct lp_fall_cmd_x;

lpfilterStruct lp_rise_cmd_y;
lpfilterStruct lp_fall_cmd_y;

static THD_WORKING_AREA(command_mixer_wa, 512);

static THD_FUNCTION(command_mixer, p)
{
    (void) p;

    dbus_msg = can_get_sent_dbus();
    ros_msg = can_get_ros_msg();

    cmd_mixer.ctrl_mode = CTL_CHASSIS_STOP;
    cmd_mixer.reboot_flag = true;

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
        float vx_dbus = (dbus_msg->channel0 - CMD_MIXER_RC_MID) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_X;
        float vy_dbus = (dbus_msg->channel1 - CMD_MIXER_RC_MID) / RC_RESOLUTION * CHASSIS_RC_MAX_SPEED_Y;

        float vx_ros  = (float)ros_msg->px; // mm/s
        float vy_ros  = (float)ros_msg->py; // mm/s

        float vx_dbus_filed = lpfilter_apply(&lp_rise_cmd_x, vx_dbus);
        float vy_dbus_filed = lpfilter_apply(&lp_rise_cmd_y, vy_dbus);

        float vx_dbus_slow = (lp_rise_cmd_x.data[1] - 2 * lp_rise_cmd_x.data[1] + vx_dbus > 0)
                             ? vx_dbus_filed : vx_dbus;
        float vy_dbus_slow = (lp_rise_cmd_y.data[1] - 2 * lp_rise_cmd_y.data[1] + vy_dbus > 0)
                             ? vy_dbus_filed : vy_dbus;

        cmd_mixer.vx = vx_ros + vx_dbus_slow;
        cmd_mixer.vy = vy_ros + vy_dbus_slow;

        /*
         * state machine
         */

        if (dbus_msg->channel0 <= CMD_MIXER_RC_MAX || dbus_msg->channel0 >= CMD_MIXER_RC_MIN ||
            dbus_msg->channel1 <= CMD_MIXER_RC_MAX || dbus_msg->channel1 >= CMD_MIXER_RC_MIN)
        {
            cmd_mixer.ctrl_mode = CTL_MANUAL_FOLLOW_GIMBAL;
            if (cmd_mixer.reboot_flag) {
                cmd_mixer.reboot_flag = false;
            }
        }
        else {
            // handle reboot
            cmd_mixer.ctrl_mode = CTL_CHASSIS_STOP;
            cmd_mixer.reboot_flag = true;
        }
    }
}

void command_mixer_init()
{
    lpfilter_init(&lp_rise_cmd_x, CMD_MIXER_UPDATE_FREQ, 5);
    lpfilter_init(&lp_rise_cmd_y, CMD_MIXER_UPDATE_FREQ, 5);
    lpfilter_init(&lp_fall_cmd_x, CMD_MIXER_UPDATE_FREQ, 30);
    lpfilter_init(&lp_fall_cmd_y, CMD_MIXER_UPDATE_FREQ, 30);
    chThdCreateStatic(command_mixer_wa, sizeof(command_mixer_wa), NORMALPRIO,
                      command_mixer, NULL);
}