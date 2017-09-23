#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "flash.h"
#include "chprintf.h"

#define SERIAL_CMD       &SD2
#define SERIAL_DATA      &SD2

void shellStart(void);

#endif
