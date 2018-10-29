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
#include "feeder.h"

/*
 * EXTI 1 CALLBACK
 * Configured for motor testing
 * Resumes MotorToggleThread
 */
static void extcb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();
  feeder_bulletOut();
  chSysUnlockFromISR();
}

/*
 * Refer to STM32F4 datasheet about EXTI channel configurations
 */

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI0
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, extcb},
    //EXTI1
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI2
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI3
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI4
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI5
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI6
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI7
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI8
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI9
    {EXT_CH_MODE_DISABLED, NULL},   //EXTI10, RMShield Pushbutton
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
  extChannelEnable(&EXTD1, 1);
}
