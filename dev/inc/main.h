#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "flash.h"
#include "chprintf.h"
#include "tft_display.h"

#define SERIAL_CMD       &SD3
#define SERIAL_DATA      &SD3

void shellStart(void);

#endif
