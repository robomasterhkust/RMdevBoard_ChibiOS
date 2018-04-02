/**
 * Beck Pang, detect if modules are offline
 * @date 20180402
 * @version 0.0
 */

#include "ch.h"
#include "detect_error_task.h"

// create a thread to process this
static THD_WORKING_AREA(detect_error_task_wa, 1024);

static THD_FUNCTION(detect_error_task, p)
{
    chRegSetThreadName("detect error task");

    while (!chThdShouldTerminateX())
    {
        // Detection frequency in 10Hz
        chThdSleepMilliseconds(100);
    }
}

/**
 *
        //Control the flashing of green LED // TODO: Shift to Error.c
        if (!(count % 25)) {
            uint32_t blink_count = count / 25;
            if (!(blink_count % 8))
                LEDB_OFF();
            if (rc->state == RC_STATE_UNINIT || rc->state == RC_STATE_LOST ||
                #ifdef RC_SAFE_LOCK
                (lock_state != RC_UNLOCKED && (blink_count % 8 < 2)) ||
                (lock_state == RC_UNLOCKED && (blink_count % 8 < 4))
#else
                (blink_count % 8 < 4)
#endif
                    )
                LEDB_TOGGLE();
        }

        count++;

 */