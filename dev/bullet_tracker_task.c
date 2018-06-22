/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "bullet_tracker_task.h"
#include "string.h"

static Bullet_Tracker_t bulletTracker;
static uint32_t stats;

/**
 * Getter function for the weight
 * @brief   Return the Weights struct
 */


Bullet_Tracker_t* bulletTracker_get(void){
	return &bulletTracker;
}

uint32_t* getBulletTrackerError(void){
	return &stats;
}

static void decryptBulletCount(Bullet_Tracker_t* w, const uint8_t *rxbuf){
    w->bullet_tracker.bulletCount = rxbuf[0];

}

/**
 * This callback is invoked when a receive buffer has been completely written.
 */

static void rxend_cb(UARTDriver *uartp)
{

    // if (Weights.rx_start_flag) {
	    chSysLockFromISR();
	    chThdResumeI(&bulletTracker.thread_handler, MSG_OK);
	    chSysUnlockFromISR();
    // } else
    //     Weights.rx_start_flag = 1;
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

#define  BULLET_TRACKER_INIT_WAIT_TIME_MS      270U // the update frequency is 100ms according to the delay time in the arduino code
static THD_WORKING_AREA(uart_bullet_tracker_thread_wa, 512);

static THD_FUNCTION(uart_bullet_tracker_thread, p)
{
    Bullet_Tracker_t *w = (Bullet_Tracker_t *) p;
    chRegSetThreadName("uart bullet tracker receiver");
	// Weights_Weightsreset(w);
    msg_t rxmsg;
    systime_t timeout = MS2ST(BULLET_TRACKER_INIT_WAIT_TIME_MS);

    while (!chThdShouldTerminateX()) {
        uartStopReceive(w->uart);
        uartStartReceive(w->uart, BULLET_TRACKER_BUFFER_SIZE, w->rxbuf);

        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&w->thread_handler, timeout);
        chSysUnlock();
        stats = (uint32_t)rxmsg;
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

    uartStart(bulletTracker.uart, &uart_cfg);
    dmaStreamRelease(bulletTracker.uart->dmatx);

    bulletTrackerReset(&bulletTracker);

    chThdCreateStatic(uart_bullet_tracker_thread_wa, sizeof(uart_bullet_tracker_thread_wa),
                      NORMALPRIO + 7,
                      uart_bullet_tracker_thread, &bulletTracker);
}
