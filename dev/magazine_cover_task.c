#include "magazine_cover_task.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "keyboard.h"
// #include <stdbool.h>
static PWMConfig pwm8cfg = {
        1000000,   /* 1MHz PWM clock frequency.   */
        20000,      /* Initial PWM period 20ms.       */
        NULL,
        {
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};
Gimbal_Send_Dbus_canStruct* PRC;
// const int LEFTCOVER = 0; // D
// const int RIGHTCOVER = 1; // C

static magCoverStruct_t magCover;
static bool inited = false;
const int LEFTCOVER = 1; // D
const int RIGHTCOVER = 2; // C

magCoverStruct_t* getMagCover(void){
  return &magCover;
}


// void magCoverClose(void){
//     pwmStop(&PWMD5);
//     pwmStart(&PWMD5,&pwm5cfg);
//     pwmEnableChannel(&PWMD5, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 5000));
//     pwmEnableChannel(&PWMD5, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 5));
// }

// void magCoverOpen(void){
//     pwmStop(&PWMD5);
//     pwmStart(&PWMD5,&pwm5cfg);
//     pwmEnableChannel(&PWMD5, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 500));
//     pwmEnableChannel(&PWMD5, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 1000));
// }

void magCoverToggle(void){
  if(!magCover.internalState){
    magCoverOpen();
    magCover.internalState = true;
  }else{
    magCoverClose();
    magCover.internalState = false;

  }
}

void magCoverClose(void){

//    pwmStop(&PWMD8);
//    pwmStart(&PWMD8,&pwm8cfg);
    // if(!internalState){
      pwmEnableChannel(&PWMD8, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 5000));
      pwmEnableChannel(&PWMD8, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 5));
    // }
}

void magCoverOpen(void){
//    pwmStop(&PWMD8);
//    pwmStart(&PWMD8,&pwm8cfg);
  // if(internalState){
    pwmEnableChannel(&PWMD8, LEFTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 500)); //800
    pwmEnableChannel(&PWMD8, RIGHTCOVER, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1000)); //750
  // }


}

static THD_WORKING_AREA(magazine_cover_wa, 2048);
static THD_FUNCTION(magazine_cover, p)
{

  (void*)p;
  chRegSetThreadName("Magazine_cover");
  PRC = can_get_sent_dbus();
  uint32_t tick = chVTGetSystemTimeX();
  while(!chThdShouldTerminateX())
  {
    tick += US2ST(Maga_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }
    if(magCover.bitmap_for_magCover[KEY_R]){
      if(!magCover.R_press){
        magCoverToggle();
      }
      magCover.R_press = true;

    }
    else{
      magCover.R_press = false;
    }

    // if(PRC->s1 == MI){
    //   magCoverOpen();
    // }
    // else{
    //   magCoverClose();
    // }


  }
}
bool getMagCoverInitStatus(void){
  return inited;
}


void pwm_magazine_cover_init(void)
{
    void pwm_config(PWMDriver *pwmp, const PWMConfig *config,int p){
        pwmStop(pwmp);
        pwmStart(pwmp,config);
        
        pwmEnableChannel(pwmp, 1, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
        pwmEnableChannel(pwmp, 2, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
        pwmEnableChannel(pwmp, 3, PWM_PERCENTAGE_TO_WIDTH(pwmp, p));
    }
    pwmStart(&PWMD8,&pwm8cfg);
    magCover.internalState = false;
    magCover.bitmap_for_magCover = Bitmap_get();
    inited = true;
    magCover.R_press = false;

    magCoverClose();
    chThdCreateStatic(magazine_cover_wa, sizeof(magazine_cover_wa),
                               NORMALPRIO, magazine_cover, NULL);
    // chThdSleepSeconds(1);
}
