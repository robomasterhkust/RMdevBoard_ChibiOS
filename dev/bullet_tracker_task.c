/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "bullet_tracker_task.h"
#include "string.h"

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


int* getBulletTrackerError(void){
    return &stats;
}

// uint32_t* getBulletTrackerError(void){
//  return &stats;
// }

static void decryptBulletCount(Bullet_Tracker_t* w, const uint8_t *rxbuf){
    w->bullet_tracker.bulletCount = rxbuf[0];
    // uint32_t temp;
    // temp = (((uint32_t)rxbuf[0]) | ((uint32_t)rxbuf[1] << 8)| ((uint32_t)rxbuf[2] << 16) |((uint32_t)rxbuf[3] << 24));
    // memcpy(&(w->weights.weight1), &temp, 8);
    // temp = (((uint32_t)rxbuf[4]) | ((uint32_t)rxbuf[5] << 8)| ((uint32_t)rxbuf[6] << 16) |((uint32_t)rxbuf[7] << 24));
    // memcpy(&(w->weights.weight2), &temp, 8);
    // temp = (((uint32_t)rxbuf[8]) | ((uint32_t)rxbuf[9] << 8)| ((uint32_t)rxbuf[10] << 16) |((uint32_t)rxbuf[11] << 24));
    // memcpy(&(w->weights.weight3), &temp, 8);
    // temp = (((uint32_t)rxbuf[12]) | ((uint32_t)rxbuf[13] << 8)| ((uint32_t)rxbuf[14] << 16) |((uint32_t)rxbuf[15] << 24));
    // memcpy(&(w->weights.weight4), &temp, 8);

}

/**
 * This callback is invoked when a receive buffer has been completely written.
 */

static void rxend_cb(UARTDriver *uartp)
{

        chSysLockFromISR();
        chThdResumeI(&bulletTracker.thread_handler, MSG_OK);
        chSysUnlockFromISR();
}


/**
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
        NULL, NULL, rxend_cb, NULL, NULL,
        100000,
        USART_CR1_PCE,
        0,
        0
};

static void bulletTrackerReset(Bullet_Tracker_t *w){
    w->bullet_tracker.bulletCount = 0;
}


#define  BULLET_TRACKER_INIT_WAIT_TIME_MS      260U // the update frequency is 100ms according to the delay time in the arduino code
static THD_WORKING_AREA(uart_bullet_tracker_thread_wa, 512);

static THD_FUNCTION(uart_bullet_tracker_thread, p)
{
    Bullet_Tracker_t *w = (Bullet_Tracker_t *) p;
    chRegSetThreadName("uart bullet tracker receiver");
    msg_t rxmsg;
    systime_t timeout = MS2ST(BULLET_TRACKER_INIT_WAIT_TIME_MS);

    while (!chThdShouldTerminateX()) {
        uartStopReceive(w->uart);
        uartStartReceive(w->uart, BULLET_TRACKER_BUFFER_SIZE, w->rxbuf);
        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&w->thread_handler, timeout);
        chSysUnlock();
        stats = (int)rxmsg;
       if (rxmsg == MSG_OK) { // to ensure that the message has been received correctly
            chSysLock();
            decryptBulletCount(w, w->rxbuf);
            chSysUnlock();
        }else{
            bulletTrackerReset(w);
            timeout = MS2ST(BULLET_TRACKER_INIT_WAIT_TIME_MS);
        }
    }
}

/**
 * @brief   Initialize the Weights receiver
 */
void magazineTracker_init(void)
{
    memset(&bulletTracker, 0, sizeof(Bullet_Tracker_t));

    bulletTracker.uart = UART_BULLET_TRACKER;
    bulletTracker.thread_handler = NULL;
    bulletTracker.inited = false;

    uartStart(bulletTracker.uart, &uart_cfg);
    dmaStreamRelease(bulletTracker.uart->dmatx);

    bulletTrackerReset(&bulletTracker);
    bulletTracker.inited = true;

    chThdCreateStatic(uart_bullet_tracker_thread_wa, sizeof(uart_bullet_tracker_thread_wa),
                      NORMALPRIO + 7,
                      uart_bullet_tracker_thread, &bulletTracker);
}
