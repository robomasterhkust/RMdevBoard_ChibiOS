#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "math_misc.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"

#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"
#include "sdlog.h"
#include "exti.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "imu_temp.h"
#include "calibrate_sensor.h"

#include "gimbal.h"
#include "shoot.h"
#include "feeder.h"
#include "barrelStatus.h"

#include "complementary_filter.h"
#include "can_comm.h"

#include "rune.h"
#include "keyboard.h"
void shellStart(void);

bool power_check(void);
bool power_failure(void);

#endif
