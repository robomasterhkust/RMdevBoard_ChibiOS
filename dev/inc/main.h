#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "error.h"
#include "sdlog.h"
#include "mavlink_comm.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "chassis.h"

#include "exti.h"
#include "judge.h"
#include "imu_temp.h"
#include "shoot_pwm.h"

#include "command_mixer.h"
#include "gimbal_simple_controller.h"

void shellStart(void);

#endif
