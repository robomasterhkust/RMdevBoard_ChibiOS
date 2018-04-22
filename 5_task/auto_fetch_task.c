//
// Created by XU Xinyuan on 22/4/2018.
//
#include "ch.h"
#include "hal.h"
#include "auto_fetch_task.h"

thread_reference_t auto_fetch_thread_ref = NULL;

static THD_WORKING_AREA(auto_fetch_thread_wa, 512);

static THD_FUNCTION(auto_fetch_thread, ptr)
{
    chThdSuspendS(&auto_fetch_thread_ref);

    chThdSleepMilliseconds(200);
}

void auto_fetch_task_init(void)
{
    chThdCreateStatic(auto_fetch_thread_wa, sizeof(auto_fetch_thread_wa),
                    NORMALPRIO, auto_fetch_thread, NULL);
}