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
#include "canBusProcess.h"
#include "can.h"

//comment out the line below to disable motor testing
#define MOTOR_TEST

/*
 * Turns on all chassis motor for 1 sec when MotorOn is TRUE
 * Thread normally suspended, resumes when shield button is pushed
 * For power module development only
 */

thread_reference_t button_thread_ref = NULL;
static volatile bool MotorOn = FALSE;
static THD_WORKING_AREA(MotorToggleThread_wa, 128);
static THD_FUNCTION(MotorToggleThread, arg) {

  (void)arg;
  chSysLock();
  while (TRUE) {

    chSysUnlock();
    if (MotorOn) {

      MotorOn = FALSE;
      palSetPad(GPIOA, GPIOA_LED_Y);
      can_motorSetCurrent(&CAND1, 0x200, 32767, 32767, 32767, 32767);
      chThdSleepMilliseconds(1000);

    }

    palClearPad(GPIOA, GPIOA_LED_Y);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);    //for some reason multiple calls
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);    //are needed to stop the motors
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    can_motorSetCurrent(&CAND1, 0x200, 0, 0, 0, 0);
    chSysLock();

    chThdSuspendS(&button_thread_ref);

  }
}

/*
 * EXTI 10 CALLBACK
 * Configured for motor testing
 * Resumes MotorToggleThread
 */
static void extcb10(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  #ifdef MOTOR_TEST
    chSysLockFromISR();
    MotorOn = TRUE;
    chThdResumeI(&button_thread_ref, MSG_OK);
    chSysUnlockFromISR();
  #endif

}

/*
 * Refer to STM32F4 datasheet about EXTI channel configurations
 */

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI0
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI1
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI2
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI3
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI4
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI5
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI6
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI7
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI8
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI9
    {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOF, extcb10},   //EXTI10, RMShield Pushbutton
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

void extiinit(void) {

  extStart(&EXTD1, &extcfg);
  extChannelEnable(&EXTD1, 10);
  chThdCreateStatic(MotorToggleThread_wa, sizeof(MotorToggleThread_wa),
                    NORMALPRIO, MotorToggleThread, NULL);

}
