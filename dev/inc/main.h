#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"
#include "tft_display.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "error.h"
#include "judge.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "calibrate_sensor.h"
#include "custom_data.h"
#include "bullet_tracker_task.h"
#include "magazine_cover_task.h"

#include "gimbal.h"
#include "chassis.h"

#include "exti.h"
#include "imu_temp.h"
#include "sdlog.h"
#include "barrelStatus.h"

void shellStart(void);

#endif
