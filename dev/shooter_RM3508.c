#include "stdint.h"
#include "shooter_rm3508.h"
#include "ch.h"
#include "hal.h"

#include "dbus.h"

RC_Ctl_t *p_dbus_shooter;

extern PWMDriver PWMD12;

static PWMConfig test_pwm12cfg = {
        500000,                                    /* 1MHz PWM clock frequency.   */
        1000,                                       /* Initial PWM period 1ms.     */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};


static THD_WORKING_AREA(shooter_wa, 512);

static THD_FUNCTION(shooter_thd, p)
{
    while (!chThdShouldTerminateX()) {
/**
 * Comment: RC control, the other is slow rotation
 */
/*if(p_dbus_shooter->rc.s2 == 1){
    pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 7200));
    pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 7200));
    //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    chThdSleepMilliseconds(200);
}
else{
    pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 5000));
    pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 5000));
    //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    chThdSleepMilliseconds(200);
}*/
        pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
        pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, 6000));
    }
}


void shooter_rm3508_init(void)
{

    pwmStart(&PWMD12, &test_pwm12cfg);

    int i = 0;
    for (i = 0; i < 120; i++) {
//        pwmStop(&PWMD12);
//        pwmStart(&PWMD12, &test_pwm12cfg);
        pwmEnableChannel(&PWMD12, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        pwmEnableChannel(&PWMD12, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        //pwmEnableChannel(&PWMD12, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        //pwmEnableChannel(&PWMD12, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD12, i * 50));
        chThdSleepMilliseconds(50);
    }

    p_dbus_shooter = RC_get();


    chThdCreateStatic(shooter_wa, sizeof(shooter_wa),
                      NORMALPRIO, shooter_thd, NULL);
}