/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "dbus.h"
#include "string.h"

static RC_Ctl_t RC_Ctl;
//static thread_reference_t uart_dbus_thread_handler = NULL;
//static dbus_error_t rxflag = false;

#ifdef RC_SAFE_LOCK
systime_t update_time;
rc_lock_state_t lock_state;
#endif

/**
 * Getter function for the remote controller
 * @brief   Return the RC_Ctl struct
 */
RC_Ctl_t *RC_get(void)
{
    return &RC_Ctl;
}

/**
 * @brief   Decode the received DBUS sequence and store it in RC_Ctl struct
 */
static void decryptDBUS(RC_Ctl_t *rc, const uint8_t *rxbuf)
{
#ifdef RC_SAFE_LOCK
    uint8_t prev_s1 = rc->rc.s1;
    uint8_t prev_s2 = rc->rc.s2;
#endif

    rc->rc.channel0 = ((rxbuf[0]) | (rxbuf[1] << 8)) & (uint16_t) 0x07FF;
    rc->rc.channel1 = ((rxbuf[1] >> 3) | (rxbuf[2] << 5)) & (uint16_t) 0x07FF;
    rc->rc.channel2 = ((rxbuf[2] >> 6) | (rxbuf[3] << 2) | ((uint16_t) rxbuf[4] << 10)) & (uint16_t) 0x07FF;
    rc->rc.channel3 = ((rxbuf[4] >> 1) | (rxbuf[5] << 7)) & (uint16_t) 0x07FF;
    rc->rc.s1 = ((rxbuf[5] >> 4) & (uint8_t) 0x000C) >> 2;     //!< Switch left
    rc->rc.s2 = ((rxbuf[5] >> 4) & (uint8_t) 0x0003);


    rc->mouse.x = rxbuf[6] | (rxbuf[7] << 8);                   //!< Mouse X axis
    rc->mouse.y = rxbuf[8] | (rxbuf[9] << 8);                   //!< Mouse Y axis
    rc->mouse.z = rxbuf[10] | (rxbuf[11] << 8);                 //!< Mouse Z axis
    rc->mouse.LEFT = rxbuf[12];                                       //!< Mouse Left Is Press ?
    rc->mouse.RIGHT = rxbuf[13];                                       //!< Mouse Right Is Press ?
    rc->keyboard.key_code = rxbuf[14] | (rxbuf[15] << 8);                   //!< KeyBoard value

#ifdef RC_SAFE_LOCK
    bool mid_flag0 = rc->rc.channel0 > (RC_CH_VALUE_OFFSET - 5) && rc->rc.channel0 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag1 = rc->rc.channel1 > (RC_CH_VALUE_OFFSET - 5) && rc->rc.channel1 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag2 = rc->rc.channel2 > (RC_CH_VALUE_OFFSET - 5) && rc->rc.channel2 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag3 = rc->rc.channel3 > (RC_CH_VALUE_OFFSET - 5) && rc->rc.channel3 < (RC_CH_VALUE_OFFSET + 5);
    if (lock_state == RC_UNLOCKED &&
        (rc->rc.channel0 != RC_CH_VALUE_OFFSET ||
         rc->rc.channel1 != RC_CH_VALUE_OFFSET ||
         rc->rc.channel2 != RC_CH_VALUE_OFFSET ||
         rc->rc.channel3 != RC_CH_VALUE_OFFSET ||
         rc->rc.s1 != prev_s1 ||
         rc->rc.s2 != prev_s2)
            )
        update_time = chVTGetSystemTimeX();
    else if (lock_state == RC_LOCKED
             && rc->rc.channel0 > RC_CH_VALUE_MAX - 5
             && rc->rc.channel1 < RC_CH_VALUE_MIN + 5
             && rc->rc.channel2 < RC_CH_VALUE_MIN + 5
             && rc->rc.channel3 < RC_CH_VALUE_MIN + 5)
        lock_state = RC_UNLOCKING;
    else if (lock_state == RC_UNLOCKING && mid_flag0 && mid_flag1 && mid_flag2 && mid_flag3) {
        update_time = chVTGetSystemTimeX();
        lock_state = RC_UNLOCKED;
    }
#endif
}

/**
 * @brief get error code: rx_flag
 * @status: true = connected
 *          false = not connected
 */
rc_state_t dbus_getError(void)
{
    return RC_get()->state;
}

/**
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend_cb(UARTDriver *uartp)
{

    if (RC_Ctl.rx_start_flag) {
        chSysLockFromISR();
        chThdResumeI(&RC_Ctl.thread_handler, MSG_OK);
        chSysUnlockFromISR();
    } else
        RC_Ctl.rx_start_flag = 1;
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

/**
 * @brief Reset RC controller
 * @NOTE  This function is also used as safe lock mechanism for RC controller
 *        S2 is not flushed because it is used to unlock the RC controller
 */
static void RC_RCreset(RC_Ctl_t *rc)
{
    rc->rc.channel0 = 1024;
    rc->rc.channel1 = 1024;
    rc->rc.channel2 = 1024;
    rc->rc.channel3 = 1024;

}

static void RC_reset(RC_Ctl_t *rc)
{
    RC_RCreset(rc);

#ifdef RC_SAFE_LOCK
    lock_state = RC_LOCKED;
#endif

    rc->rc.s1 = 0;
    rc->rc.s2 = 0;
    rc->mouse.LEFT = 0;
    rc->mouse.RIGHT = 0;
    rc->mouse.x = 0;
    rc->mouse.y = 0;
    rc->mouse.z = 0;
    rc->keyboard.key_code = 0;
}

#define  DBUS_INIT_WAIT_TIME_MS      4U
#define  DBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_dbus_thread_wa, 512);

static THD_FUNCTION(uart_dbus_thread, p)
{
    RC_Ctl_t *rc = (RC_Ctl_t *) p;
    chRegSetThreadName("uart dbus receiver");

    msg_t rxmsg;
    systime_t timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);

    rc->state = RC_STATE_LOST;

    while (!chThdShouldTerminateX()) {
        uartStopReceive(rc->uart);
        uartStartReceive(rc->uart, DBUS_BUFFER_SIZE, rc->rxbuf);

        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&rc->thread_handler, timeout);
        chSysUnlock();

        if (rxmsg == MSG_OK) {
            if (rc->state == RC_STATE_LOST) {
                timeout = MS2ST(DBUS_WAIT_TIME_MS);
                rc->state = RC_STATE_CONNECTED;
            } else {
                chSysLock();
                decryptDBUS(rc, rc->rxbuf);

#ifdef RC_SAFE_LOCK
                if (lock_state != RC_UNLOCKED)
                    RC_RCreset(rc);
                else if (chVTGetSystemTimeX() > update_time + S2ST(RC_LOCK_TIME_S))
                    lock_state = RC_LOCKED;
#endif

                chSysUnlock();
            }
        } else {
            rc->state = RC_STATE_LOST;
            RC_reset(rc);
            timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
        }
    }
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void)
{
    memset(&RC_Ctl, 0, sizeof(RC_Ctl_t));

    RC_Ctl.uart = UART_DBUS;
    RC_Ctl.thread_handler = NULL;

    uartStart(RC_Ctl.uart, &uart_cfg);
    dmaStreamRelease(RC_Ctl.uart->dmatx);

    RC_reset(&RC_Ctl);

    chThdCreateStatic(uart_dbus_thread_wa, sizeof(uart_dbus_thread_wa),
                      NORMALPRIO + 7,
                      uart_dbus_thread, &RC_Ctl);
}