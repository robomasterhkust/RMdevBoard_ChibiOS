/*
 * exti.c
 *
 *  Created on: 24 Dec 2017
 *      Author: Alex Wong
 *
 *  Configures External Interrupt Functionality
 *
 */

#include "ch.h"
#include "hal.h"
#include "exti.h"
#include "can_motor_task.h"
#include "can.h"
#include "adis16470.h"

//comment out the line below to disable motor testing

#define DEBOUNCE_TIME 100

//extern thread_reference_t imu_adis_thread_ref;
extern thread_reference_t auto_fetch_thread_ref;

static int count = 0;
static systime_t last_exti_0_time;

/**
 * @brief EXTI 0 CALLBACK with debouncing
 *          Configured for the hero switch to trigger the auto fetching function
 *          Since the switch is noisy, debounce for 10 ms
 */
static void ext_switch_cb0(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;
    (void) channel;

    // Debouncing for the mechanical swtich
    if (chVTGetSystemTimeX() > last_exti_0_time + MS2ST(DEBOUNCE_TIME))
    {
        last_exti_0_time = chVTGetSystemTimeX();

        // LEDG3_TOGGLE();
        chSysLockFromISR();
        chThdResumeI(&auto_fetch_thread_ref, MSG_OK);
        chSysUnlockFromISR();
    }
}

/*
 * EXTI 1 CALLBACK
 * Configured for the future
 */
static void ext_switch_cb1(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;
    (void) channel;

//    chSysLockFromISR();
//    chThdResumeI(&auto_fetch_thread_ref, MSG_OK);
//    chSysUnlockFromISR();
}

/*
 * EXTI 2 CALLBACK
 * Configured for the button on the RM2018 board
 */
static void ext_key_cb2(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;
    (void) channel;

    chSysLockFromISR();
    LEDG_TOGGLE(); // Changed to other functions
    chSysUnlockFromISR();
}

/*
 * EXTI 3 CALLBACK
 * Configured for the future
 */
static void ext_switch_cb3(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;
    (void) channel;

//    chSysLockFromISR();
//    chThdResumeI(&auto_fetch_thread_ref, MSG_OK);
//    chSysUnlockFromISR();
}

/*
 * EXTI 5 CALLBACK
 * Configured for ADIS16470 IMU data ready pin
 */
static void extcb5(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;
    (void) channel;

    count++;
    if (count % 10 == 0) {
        chSysLockFromISR();
//        chThdResumeI(&imu_adis_thread_ref, MSG_OK);
        chSysUnlockFromISR();
    }
}

/*
 * EXTI 10 CALLBACK
 * Configured for motor testing
 * Resumes MotorToggleThread
 */
static void extcb10(EXTDriver *extp, expchannel_t channel)
{

    (void) extp;
    (void) channel;

}

/*
 * Refer to STM32F4 datasheet about EXTI channel configurations
 */

static const EXTConfig extcfg = {
        {
                {EXT_CH_MODE_DISABLED, NULL},
                {EXT_CH_MODE_DISABLED, NULL},
                {EXT_CH_MODE_DISABLED, NULL},
//                {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART |
//                 EXT_MODE_GPIOA, ext_switch_cb0},   //EXTI0
//                {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART |
//                 EXT_MODE_GPIOA, ext_switch_cb1},   //EXTI1
//                {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART |
//                 EXT_MODE_GPIOB, ext_key_cb2},   //EXTI2
                {EXT_CH_MODE_DISABLED, NULL},
//                {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART |
//                 EXT_MODE_GPIOA, ext_switch_cb3},   //EXTI3
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI4
                {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART |
                 EXT_MODE_GPIOC, extcb5},   //EXTI5, ADIS shield
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI6
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI7
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI8
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI9
//                {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART |
//                 EXT_MODE_GPIOF, extcb10},   //EXTI10, RMShield Pushbutton
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI10
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI11
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI12
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI13
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI14
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI15
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI16, PVD OUTPUT
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI17, RTC ALARM
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI18, USB OTG FS WAKEUP
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI19, ETH WAKEUP
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI20, USB OTG hS WAKEUP
                {EXT_CH_MODE_DISABLED, NULL},   //EXTI21, RTC TAMPER/TimeStamp
                {EXT_CH_MODE_DISABLED, NULL}    //EXTI22, RTC WAKEUP
        }
};

void extiinit(void)
{

    extStart(&EXTD1, &extcfg);
//    extChannelEnable(&EXTD1, 0);
//    last_exti_0_time = chVTGetSystemTimeX();

//    extChannelEnable(&EXTD1, 2);
//    extChannelEnable(&EXTD1, 5);
//    extChannelEnable(&EXTD1, 10);

}
