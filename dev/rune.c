#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "feeder.h"
#include "gimbal.h"
#include "rune.h"
#include "shoot.h"
#include "keyboard.h"
#include "canBusProcess.h"
#include "feeder.h"

static GimbalStruct *gimbal;
static volatile Rune_canStruct *rune_can;
static PIMUStruct pIMU;
static RC_Ctl_t* rc;
static bool rune_state = false;

bool rune_remote_control_enable = false;
static bool G_press = false;

static bool rune_can_updated = false;
static uint8_t* feeder_fire_mode;
static feeder_mode_t* feeder_mode;

void rune_cmd(uint8_t cmd) {
#ifdef RUNE_FIRE_SAFE
// if()
//  shooter_control(RUNE_FIRE_POWER);
// else
//  shooter_control(0);
#endif
  rune_state = cmd == DISABLE ? false : true;
  gimbal_setRune(cmd);
}

void rune_fire(const float yaw, const float pitch, bool fire) {
  gimbal->pitch_atti_cmd = pitch;
  gimbal->yaw_atti_cmd = yaw + (pIMU->euler_angle[Yaw] - gimbal->d_yaw);

  float pitch_error = gimbal->pitch_atti_cmd - pIMU->euler_angle[Pitch],
        yaw_error = gimbal->yaw_atti_cmd - pIMU->euler_angle[Yaw];

  uint8_t count = 0;
  while (count++ < 8) {
    if (pitch_error > RUNE_MAX_ERROR || pitch_error < -RUNE_MAX_ERROR ||
        yaw_error > RUNE_MAX_ERROR || yaw_error < -RUNE_MAX_ERROR)
      count = 0;

    pitch_error = gimbal->pitch_atti_cmd - pIMU->euler_angle[Pitch];
    yaw_error = gimbal->yaw_atti_cmd - pIMU->euler_angle[Yaw];

    chThdSleepMilliseconds(5);
  }
  if (fire){
    feeder_singleShot();
  }
  gimbal_Follow();
}

static THD_WORKING_AREA(rune_wa, 256);
static THD_FUNCTION(rune_thread, p) {
  (void)p;
  while (!chThdShouldTerminateX()) {
    // should be modified to keyboard "g"
    if(bitmap[KEY_G]){
      if(!G_press){
        if(rune_remote_control_enable){
          rune_fire(0.0f, 0.0f, false);
          rune_remote_control_enable = false;
        }
        else{
          rune_fire(0.0f, 0.0f, true);
          rune_remote_control_enable = true;
        }

      }
      G_press = true;
    }
    else{
      G_press = false;
    }

    // //
    // systime_t now = chVTGetSystemTimeX();
    // if (now > MS2ST(500) + rune_can->last_time){
    //   rune_can_updated = true;
    // }else{
    //   rune_can_updated = false;
    // }

    if(rune_remote_control_enable && rune_can->updated){
      rune_cmd(ENABLE);
      rune_fire(rune_can->pz, rune_can->py, true);
      rune_cmd(DISABLE);
      rune_can->updated = false;
    }
    else
      chThdSleep(RUNE_THREAD_PERIOD);
  }
}

void rune_init(void) {
  gimbal = gimbal_get();
  rune_can = can_get_rune();
  pIMU = imu_get();
  rc = RC_get();
  // feeder_fire_mode = get_feeder_fire_mode();
  // feeder_mode = get_feeder();
  chThdCreateStatic(rune_wa, sizeof(rune_wa), NORMALPRIO - 5, rune_thread,
                    NULL);
}
