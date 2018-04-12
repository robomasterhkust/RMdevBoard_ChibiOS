#include "stdint.h"
#include "ch.h"
#include "hal.h"

#include "bullet_count_task.h"
//#include "canBusProcess.h"


int bullet_count = 0;

extern PWMDriver PWMD4;

static PWMConfig test_pwm4cfg = {
        500000,                                    /* 1MHz PWM clock frequency.   */
        1000,                                       /* Initial PWM period 1ms.     */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

static THD_WORKING_AREA(bullet_count_wa, 512);

static THD_FUNCTION(bullet_count_thd, p)
{

    pwmStart(&PWMD4, &test_pwm4cfg);

    int i = 0;
    for (i = 0; i < 147; i++) {
        pwmStop(&PWMD4);
        pwmStart(&PWMD4, &test_pwm4cfg);
        pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, i * 50));
        //pwmEnableChannel(&PWMD4, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, i * 50));
        //pwmEnableChannel(&PWMD4, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, i * 50));
        //pwmEnableChannel(&PWMD4, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, i * 50));
        chThdSleepMilliseconds(50);
    }

    uint32_t last_in_detect_bullet = 0;
    uint32_t current_in_detect_bullet = 0;
    uint32_t last_out_detect_bullet = 0;
    uint32_t current_out_detect_bullet = 0;
    uint8_t bullet_pass = 0;
    systime_t count_tick = chVTGetSystemTime();
    int8_t error_start = 0;
    systime_t error_start_time;
    int16_t set_speed = 7500 - 150;
    while (!chThdShouldTerminateX()) {

        last_in_detect_bullet = current_in_detect_bullet;
        current_in_detect_bullet = palReadPad(GPIOA, GPIOA_PIN4);
        last_out_detect_bullet = current_out_detect_bullet;
        current_out_detect_bullet = palReadPad(GPIOI, GPIOI_PIN9);

        if (current_out_detect_bullet == 1 && last_out_detect_bullet == 0) {
            bullet_count--;
        }

        if (current_in_detect_bullet == 1 && last_in_detect_bullet == 0 && (set_speed == 7885 - 150)) {
            bullet_count++;
            bullet_pass = 1;
            count_tick = chVTGetSystemTime();
        } else {
            bullet_pass = 0;
        }

        if (bullet_pass == 0 && (ST2MS(chVTGetSystemTime()) - (ST2MS(count_tick)) > 4000) &&
            ST2MS(chVTGetSystemTime()) - (ST2MS(error_start_time)) > 4000) {
            error_start_time = chVTGetSystemTime();
            error_start = 1;
        }

        if (bullet_count < 5) {
            if (error_start == 1 && (ST2MS(chVTGetSystemTime()) - (ST2MS(error_start_time)) < 2000)) {
                set_speed = 7114 - 150;
                pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, set_speed));
            } else {
                set_speed = 7885 - 150;
                pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, set_speed));
            }
        } else {
            set_speed = 7500 - 150;
            pwmEnableChannel(&PWMD4, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD4, set_speed));
        }
        chThdSleepMilliseconds(1);
    }
}

void bullet_count_task_init(void)
{
    chThdCreateStatic(bullet_count_wa, sizeof(bullet_count_wa),
                      NORMALPRIO + 1, bullet_count_thd, NULL);
}