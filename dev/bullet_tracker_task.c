

#include "ch.h"
#include "hal.h"
// #include "judge.h"
#include <string.h>
#include <stdbool.h>

#define BULLET_WEIGHT 0.1688

#include "bullet_tracker_task.h"

static Bullet_Tracker_t bulletTracker;
// static uint32_t stats;
static int stats;

/**
 * Getter function for the weight
 * @brief   Return the Weights struct
 */


Bullet_Tracker_t* bulletTracker_get(void){
    return &bulletTracker;
}

/**********************************  CRC END  ********************************/


#define FLUSH_I_QUEUE(sdp)      \
    chSysLock();                \
    chIQResetI(&(sdp)->iqueue); \
    chSysUnlock();                          //Flush serial in queue

#define LEAST_SET_BIT(x)        x&(-x)      //Clear all but least set bit

#define WEIGHTACQTIME            6           //Milliseconds

#define SERIAL_EVT_MASK         1

/*
 * certain data bits enforced by chibios, check serial_lld.c for forced set bits
 * to avoid unexpected settings of UART driver
 */
static const SerialConfig SERIAL_WEIGHT_CONFIG = {
  115200,               //Baud Rate
  USART_CR1_UE,         //CR1 Register
  USART_CR2_LINEN,      //CR2 Register
  0                     //CR3 Register
};

static mutex_t inqueue_mutex;
static uint8_t foundheader = 0;
static uint8_t headerloc = 0;
static uint8_t datalength = 0;
static uint8_t packetdatalength = 0;
static uint8_t sdrxbuf[WEIGHT_BUFFER_SIZE];


bool getWeightInitStatus(void){
  return bulletTracker.inited;
}



/*
 * Handles receiving of UART data from judge
 */
static THD_WORKING_AREA(WeightThread_wa, 1024);
static THD_FUNCTION(WeightThread, arg) {

  (void)arg;
  chRegSetThreadName("Weight receiver");

  memset((void *)sdrxbuf, 0, WEIGHT_BUFFER_SIZE);

  chMtxObjectInit(&inqueue_mutex);

  static const eventflags_t serial_wkup_flags =                     //Partially from SD driver
    CHN_INPUT_AVAILABLE | CHN_DISCONNECTED | SD_NOISE_ERROR |       //Partially inherited from IO queue driver
    SD_PARITY_ERROR | SD_FRAMING_ERROR | SD_OVERRUN_ERROR |
    SD_BREAK_DETECTED;

  event_listener_t serial_listener;
  static eventflags_t pending_flags;
  static eventflags_t current_flag;
  chEvtRegisterMaskWithFlags(chnGetEventSource(SERIAL_WEIGHT), &serial_listener,
                             SERIAL_EVT_MASK, serial_wkup_flags);   //setup event listening

  while (!chThdShouldTerminateX()) {


    chEvtWaitAny(1);                                                //wait for selected serial events
    chSysLock();
    pending_flags = chEvtGetAndClearFlagsI(&serial_listener);       //get event flag
    chSysUnlock();
    foundheader = false;

    do {

      current_flag = LEAST_SET_BIT(pending_flags);                  //isolates single flag to work on
      pending_flags &= ~current_flag;                               //removes isolated flag

      switch(current_flag) {

      case CHN_INPUT_AVAILABLE:                                     //Serial data available
        chThdSleep(MS2ST(WEIGHTACQTIME));                           //Acquire data packet, release CPU
        if((!pending_flags)) {
          chMtxLock(&inqueue_mutex);                                  //Operation non-atomic, lock resource
          datalength = sdAsynchronousRead(SERIAL_WEIGHT, &sdrxbuf,
                                         (size_t)WEIGHT_BUFFER_SIZE);  //Non-blocking data read
          chMtxUnlock(&inqueue_mutex);                                //Release resource
          weightdecode();
          if(bulletTracker.inited){
            updateBulletCount();
          }else{
            weightOffsetInit();
          }
          
        }

        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case CHN_DISCONNECTED:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case SD_NOISE_ERROR:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case SD_PARITY_ERROR:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case SD_FRAMING_ERROR:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case SD_OVERRUN_ERROR:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      case SD_BREAK_DETECTED:
        FLUSH_I_QUEUE(SERIAL_WEIGHT);
        break;

      default:
        break;

      }

    } while (pending_flags && !foundheader);

    FLUSH_I_QUEUE(SERIAL_WEIGHT);
    memset((void*)sdrxbuf, 0, WEIGHT_BUFFER_SIZE);               //Flush RX buffer
  }

}

void weightdecode() {

      memcpy((void*) &bulletTracker.bullet_tracker.weight,  //Gotta love pointers
             (void*) sdrxbuf,
             4); 

}

void updateBulletCount(void){
    bulletTracker.bullet_tracker.bulletCount = (bulletTracker.bullet_tracker.weight - bulletTracker.bullet_tracker.weightOffset) / BULLET_WEIGHT;
}

void weightOffsetInit(void){
    memcpy((void*) &bulletTracker.bullet_tracker.weightOffset,  //Gotta love pointers
     (void*) sdrxbuf,
     4);
     bulletTracker.inited = true; 
}

void weightdatainit(void) {
    memset(&bulletTracker, 0, sizeof(Bullet_Tracker_t));

    // bulletTracker.uart = UART_BULLET_TRACKER;
    bulletTracker.thread_handler = NULL;
    bulletTracker.inited = false;
    bulletTracker.bullet_tracker.weight = 0.0f;
    bulletTracker.bullet_tracker.weightOffset = -1.0f;


    // uartStart(bulletTracker.uart, &uart_cfg);
    // dmaStreamRelease(bulletTracker.uart->dmatx);

    // bulletTrackerReset(&bulletTracker);
    bulletTracker.inited = false;

}


void weightinit(void) {

  weightdatainit();
    if(!bulletTracker.inited){
      chThdSleepSeconds(3);
    } 
  sdStart(SERIAL_WEIGHT, &SERIAL_WEIGHT_CONFIG);                  //Start Serial Driver

  chThdCreateStatic(WeightThread_wa, sizeof(WeightThread_wa),     //Start Judge RX thread
                    NORMALPRIO + 5, WeightThread, NULL);
/*
  chThdCreateStatic(custom_data_thread_wa, sizeof(custom_data_thread_wa),
                NORMALPRIO + 7,
                custom_data_thread, NULL);
*/
}

/* --------------- UART 3 -----------------*/
// int* getBulletTrackerError(void){
//     return &stats;
// }

// // uint32_t* getBulletTrackerError(void){
// //  return &stats;
// // }

// static void decryptBulletCount(Bullet_Tracker_t* w, const uint8_t *rxbuf){
//     w->bullet_tracker.bulletCount = rxbuf[0];
//     // uint32_t temp;
//     // temp = (((uint32_t)rxbuf[0]) | ((uint32_t)rxbuf[1] << 8)| ((uint32_t)rxbuf[2] << 16) |((uint32_t)rxbuf[3] << 24));
//     // memcpy(&(w->weights.weight1), &temp, 8);
//     // temp = (((uint32_t)rxbuf[4]) | ((uint32_t)rxbuf[5] << 8)| ((uint32_t)rxbuf[6] << 16) |((uint32_t)rxbuf[7] << 24));
//     // memcpy(&(w->weights.weight2), &temp, 8);
//     // temp = (((uint32_t)rxbuf[8]) | ((uint32_t)rxbuf[9] << 8)| ((uint32_t)rxbuf[10] << 16) |((uint32_t)rxbuf[11] << 24));
//     // memcpy(&(w->weights.weight3), &temp, 8);
//     // temp = (((uint32_t)rxbuf[12]) | ((uint32_t)rxbuf[13] << 8)| ((uint32_t)rxbuf[14] << 16) |((uint32_t)rxbuf[15] << 24));
//     // memcpy(&(w->weights.weight4), &temp, 8);

// }

// /**
//  * This callback is invoked when a receive buffer has been completely written.
//  */

// static void rxend_cb(UARTDriver *uartp)
// {

//         chSysLockFromISR();
//         chThdResumeI(&bulletTracker.thread_handler, MSG_OK);
//         chSysUnlockFromISR();
// }


// /**
//  * UART driver configuration structure.
//  */
// static UARTConfig uart_cfg = {
//         NULL, NULL, rxend_cb, NULL, NULL,
//         100000,
//         USART_CR1_PCE,
//         0,
//         0
// };

// static void bulletTrackerReset(Bullet_Tracker_t *w){
//     w->bullet_tracker.bulletCount = 0;
// }


// #define  BULLET_TRACKER_INIT_WAIT_TIME_MS      260U // the update frequency is 100ms according to the delay time in the arduino code
// static THD_WORKING_AREA(uart_bullet_tracker_thread_wa, 512);

// static THD_FUNCTION(uart_bullet_tracker_thread, p)
// {
//     Bullet_Tracker_t *w = (Bullet_Tracker_t *) p;
//     chRegSetThreadName("uart bullet tracker receiver");
//     msg_t rxmsg;
//     systime_t timeout = MS2ST(BULLET_TRACKER_INIT_WAIT_TIME_MS);

//     while (!chThdShouldTerminateX()) {
//         uartStopReceive(w->uart);
//         uartStartReceive(w->uart, BULLET_TRACKER_BUFFER_SIZE, w->rxbuf);
//         chSysLock();
//         rxmsg = chThdSuspendTimeoutS(&w->thread_handler, timeout);
//         chSysUnlock();
//         stats = (int)rxmsg;
//        if (rxmsg == MSG_OK) { // to ensure that the message has been received correctly
//             chSysLock();
//             decryptBulletCount(w, w->rxbuf);
//             chSysUnlock();
//         }else{
//             bulletTrackerReset(w);
//             timeout = MS2ST(BULLET_TRACKER_INIT_WAIT_TIME_MS);
//         }
//     }
// }

// /**
//  * @brief   Initialize the Weights receiver
//  */
// void magazineTracker_init(void)
// {
//     memset(&bulletTracker, 0, sizeof(Bullet_Tracker_t));

//     bulletTracker.uart = UART_BULLET_TRACKER;
//     bulletTracker.thread_handler = NULL;
//     bulletTracker.inited = false;

//     uartStart(bulletTracker.uart, &uart_cfg);
//     dmaStreamRelease(bulletTracker.uart->dmatx);

//     bulletTrackerReset(&bulletTracker);
//     bulletTracker.inited = true;

//     chThdCreateStatic(uart_bullet_tracker_thread_wa, sizeof(uart_bullet_tracker_thread_wa),
//                       NORMALPRIO + 7,
//                       uart_bullet_tracker_thread, &bulletTracker);
// }

