/**
 * Beck Pang, detect if modules are offline
 * @date 20180402
 * @version 0.0
 */

#include "ch.h"
#include "main.h"
#include "detect_error_task.h"
/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg =
        {
                STM32_IWDG_PR_64,
                STM32_IWDG_RL(1000)
        };


/**
  *   @brief Check whether the 24V power is on
  */
bool is_motor_power_on(void)
{
    volatile GimbalEncoder_canStruct* can = can_getGimbalMotor();

    return can->updated;
}

/**
  *   @brief  Monitor the case of a failure on 24V power, indicating the vehicle being killed
  */
bool power_failure(void)
{
    uint32_t error = gimbal_get_error();

    return (error & (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED)) ==
           (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED);
}

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

void detect_error_task_init(void)
{
    wdgStart(&WDGD1, &wdgcfg); //Start the watchdog
//    chThdCreateStatic(detect_error_task_wa, sizeof(detect_error_task_wa),
//                      NORMALPRIO, detect_error_task, NULL);
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