//
// Created by beck on 16/12/2017.
//

#include "stdint.h"
#include "ch.h"
#include "hal.h"

#include "shoot.h"
#include "dbus.h"
#include "keyboard.h"
#include "feeder.h"

#define MIN_SHOOT_SPEED 100U
#define MAX_SHOOT_SPEED 900U

RC_Ctl_t* rc;

static uint16_t speed_sp = 0;
static bool safe = false;
static speed_mode_t speed_mode;
static uint8_t shooting_speed = 0;
static PWMDriver PWMD12;
void pwm12_setWidth(uint16_t width)
{
  PWMD12.tim->CCR[0] = width;
  PWMD12.tim->CCR[1] = width;
}

/**
 * 2017/12/17 PWM test
 * @return
 */
void shooter_control(uint16_t setpoint)
{
  if(setpoint > MAX_SHOOT_SPEED)
    setpoint = MAX_SHOOT_SPEED;
  else if(setpoint < MIN_SHOOT_SPEED)
    setpoint = MIN_SHOOT_SPEED;

  if(safe || setpoint <= MIN_SHOOT_SPEED)
    speed_sp = setpoint;
}

static const PWMConfig pwm12cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.    width   */
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

#if defined (RM_INFANTRY)
static void shooter_txCan(const uint16_t shoot_speed, const uint16_t feeder_rps)
{
  CANTxFrame txmsg;
  dbus_tx_canStruct txCan;

  txmsg.IDE = CAN_IDE_STD;
  txmsg.SID = SHOOTER_SID;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 0x08;

  chSysLock();

  txmsg.data16[0] = shoot_speed;
  txmsg.data16[1] = feeder_rps;

  chSysUnlock();

  canTransmit(SHOOTER_CAN, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
}
#endif

static THD_WORKING_AREA(pwm_thd_wa, 512);
static THD_FUNCTION(pwm_thd, arg) {
    (void)arg;

    const float alpha = 0.004f;
    float speed = 0;
    uint8_t  prev_s2  = RC_S_DUMMY;
    bool     Z_press = false;

    const uint16_t shooting_speed_minigun = speed_mode.slow_speed;

    while (!chThdShouldTerminateX())
    {
      if(rc->rc.s2 == RC_S_DOWN)
        safe = true; //Release the weapon safe

      switch (rc->rc.s1){
        case RC_S_UP:
        {
          #ifdef SHOOTER_USE_RC
          if(prev_s2 != rc->rc.s2)
          {
            switch (rc->rc.s2) {
              case RC_S_UP:
                shooting_speed = speed_mode.fast_speed;
               // shooter_control(speed_mode.fast_speed);
                break;
              case RC_S_MIDDLE:
                shooting_speed = speed_mode.slow_speed;
                //shooter_control(speed_mode.slow_speed);
                break;
              case RC_S_DOWN:
                shooting_speed = speed_mode.stop;
                //shooter_control(speed_mode.stop);
                break;
            }
          }
          prev_s2 = rc->rc.s2;
          #endif
        }break;
        case RC_S_MIDDLE:
        {
          if(bitmap[KEY_Z])
          {
            if(!Z_press)
            {
              if(shooting_speed == speed_mode.slow_speed)
                shooting_speed = speed_mode.fast_speed;
              else
                shooting_speed = speed_mode.slow_speed;
            }
            Z_press = true;
          }
          else
            Z_press = false;
        }break;
      }

      if(feeder_getSpeed() >= FEEDER_MINIGUN_RPS)
        shooting_speed = speed_mode.slow_speed;

      shooter_control(shooting_speed);
      speed = alpha * (float)speed_sp + (1-alpha) * speed;
      pwm12_setWidth((uint16_t)speed);

      #if defined (RM_INFANTRY)
        uint8_t display_speed = 0;
        if(shooting_speed == speed_mode.fast_speed)
          display_speed = 25;
        else if(shooting_speed == speed_mode.slow_speed)
          display_speed = 15;
        else
          display_speed = 0;

        shooter_txCan(display_speed , feeder_getSpeed());
      #endif

      chThdSleepMilliseconds(5);
    }
}

static void pwm12_start(void)
{
  PWMD12.tim = STM32_TIM12;
  PWMD12.channels = 2;

  uint32_t psc;
  uint32_t ccer;
  rccEnableTIM12(FALSE);
  rccResetTIM12();

  PWMD12.clock = STM32_TIMCLK1;

  PWMD12.tim->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
                     STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;

  psc = (PWMD12.clock / pwm12cfg.frequency) - 1;

  PWMD12.tim->PSC  = psc;
  PWMD12.tim->ARR  = pwm12cfg.period - 1;
  PWMD12.tim->CR2  = pwm12cfg.cr2;
  PWMD12.period = pwm12cfg.period;

  ccer = 0;
  switch (pwm12cfg.channels[0].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= STM32_TIM_CCER_CC1P;
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= STM32_TIM_CCER_CC1E;
  default:
    ;
  }
  switch (pwm12cfg.channels[1].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= STM32_TIM_CCER_CC2P;
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= STM32_TIM_CCER_CC2E;
  default:
    ;
  }

  PWMD12.tim->CCER  = ccer;
  PWMD12.tim->SR    = 0;

  PWMD12.tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

  PWMD12.state = PWM_READY;
}

void shooter_init(void)
{
    rc = RC_get();
    pwm12_start();
    speed_mode.fast_speed=170;
    speed_mode.slow_speed=117;
    speed_mode.stop = 100;
    #ifndef SHOOTER_SETUP
      pwm12_setWidth(900);
      chThdSleepSeconds(3);

      pwm12_setWidth(100);
      chThdSleepSeconds(3);

      chThdCreateStatic(pwm_thd_wa, sizeof(pwm_thd_wa), NORMALPRIO + 1, pwm_thd, NULL);
    #endif
}
