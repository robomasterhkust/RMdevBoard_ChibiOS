/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
//#include "chprintf.h"
//static BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_CMD;

static uint8_t rxbuf[DBUS_BUFFER_SIZE];
static RC_Ctl_t RC_Ctl;
static thread_reference_t uart_dbus_thread_handler = NULL;
static uint8_t rx_start_flag = 1;
static dbus_error_t rxflag = false;

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
static void decryptDBUS(void)
{
#ifdef RC_SAFE_LOCK
    uint8_t prev_s1 = RC_Ctl.rc.s1;
    uint8_t prev_s2 = RC_Ctl.rc.s2;
#endif

    RC_Ctl.rc.channel0 = ((rxbuf[0]) | (rxbuf[1] << 8)) & (uint16_t) 0x07FF;
    RC_Ctl.rc.channel1 = ((rxbuf[1] >> 3) | (rxbuf[2] << 5)) & (uint16_t) 0x07FF;
    RC_Ctl.rc.channel2 = ((rxbuf[2] >> 6) | (rxbuf[3] << 2) | ((uint16_t) rxbuf[4] << 10)) & (uint16_t) 0x07FF;
    RC_Ctl.rc.channel3 = ((rxbuf[4] >> 1) | (rxbuf[5] << 7)) & (uint16_t) 0x07FF;
    RC_Ctl.rc.s1 = ((rxbuf[5] >> 4) & (uint8_t) 0x000C) >> 2;     //!< Switch left
    RC_Ctl.rc.s2 = ((rxbuf[5] >> 4) & (uint8_t) 0x0003);


    RC_Ctl.mouse.x = rxbuf[6] | (rxbuf[7] << 8);                   //!< Mouse X axis
    RC_Ctl.mouse.y = rxbuf[8] | (rxbuf[9] << 8);                   //!< Mouse Y axis
    RC_Ctl.mouse.z = rxbuf[10] | (rxbuf[11] << 8);                 //!< Mouse Z axis
    RC_Ctl.mouse.LEFT = rxbuf[12];                                       //!< Mouse Left Is Press ?
    RC_Ctl.mouse.RIGHT = rxbuf[13];                                       //!< Mouse Right Is Press ?
    RC_Ctl.keyboard.key_code = rxbuf[14] | (rxbuf[15] << 8);                   //!< KeyBoard value

#ifdef RC_SAFE_LOCK
    bool mid_flag0 = RC_Ctl.rc.channel0 > (RC_CH_VALUE_OFFSET - 5) && RC_Ctl.rc.channel0 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag1 = RC_Ctl.rc.channel1 > (RC_CH_VALUE_OFFSET - 5) && RC_Ctl.rc.channel1 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag2 = RC_Ctl.rc.channel2 > (RC_CH_VALUE_OFFSET - 5) && RC_Ctl.rc.channel2 < (RC_CH_VALUE_OFFSET + 5);
    bool mid_flag3 = RC_Ctl.rc.channel3 > (RC_CH_VALUE_OFFSET - 5) && RC_Ctl.rc.channel3 < (RC_CH_VALUE_OFFSET + 5);
    if (lock_state == RC_UNLOCKED &&
        (RC_Ctl.rc.channel0 != RC_CH_VALUE_OFFSET ||
         RC_Ctl.rc.channel1 != RC_CH_VALUE_OFFSET ||
         RC_Ctl.rc.channel2 != RC_CH_VALUE_OFFSET ||
         RC_Ctl.rc.channel3 != RC_CH_VALUE_OFFSET ||
         RC_Ctl.rc.s1 != prev_s1 ||
         RC_Ctl.rc.s2 != prev_s2)
            )
        update_time = chVTGetSystemTimeX();
    else if (lock_state == RC_LOCKED
             && RC_Ctl.rc.channel0 > RC_CH_VALUE_MAX - 5
             && RC_Ctl.rc.channel1 < RC_CH_VALUE_MIN + 5
             && RC_Ctl.rc.channel2 < RC_CH_VALUE_MIN + 5
             && RC_Ctl.rc.channel3 < RC_CH_VALUE_MIN + 5)
        lock_state = RC_UNLOCKING;
    else if (lock_state == RC_UNLOCKING && mid_flag0 && mid_flag1 && mid_flag2 && mid_flag3 )
    {
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
dbus_error_t dbus_getError(void){
    return rxflag;
}

/**
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend_cb(UARTDriver *uartp)
{

    if (rx_start_flag) {
        chSysLockFromISR();
        chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
        chSysUnlockFromISR();
    } else
        rx_start_flag = 1;
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
static void RC_RCreset(void)
{
    RC_Ctl.rc.channel0 = 1024;
    RC_Ctl.rc.channel1 = 1024;
    RC_Ctl.rc.channel2 = 1024;
    RC_Ctl.rc.channel3 = 1024;

}

static void RC_reset(void)
{
    RC_RCreset();

#ifdef RC_SAFE_LOCK
    lock_state = RC_LOCKED;
#endif

    RC_Ctl.rc.s1 = 0;
    RC_Ctl.rc.s2 = 0;
    RC_Ctl.mouse.LEFT = 0;
    RC_Ctl.mouse.RIGHT = 0;
    RC_Ctl.mouse.x = 0;
    RC_Ctl.mouse.y = 0;
    RC_Ctl.mouse.z = 0;
    RC_Ctl.keyboard.key_code = 0;
}

#define  DBUS_INIT_WAIT_TIME_MS      4U
#define  DBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_dbus_thread_wa, 512);

static THD_FUNCTION(uart_dbus_thread, p)
{
    (void) p;
    chRegSetThreadName("uart dbus receiver");

    uartStart(UART_DBUS, &uart_cfg);
    dmaStreamRelease(*UART_DBUS.dmatx);

    size_t rx_size;
    msg_t rxmsg;
    bool rxflag = false;
    systime_t timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
    uint32_t count = 0;

    while (!chThdShouldTerminateX()) {
        uartStopReceive(UART_DBUS);
        uartStartReceive(UART_DBUS, DBUS_BUFFER_SIZE, rxbuf);

        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&uart_dbus_thread_handler, timeout);
        chSysUnlock();

        if (rxmsg == MSG_OK) {
            if (!rxflag) {
                timeout = MS2ST(DBUS_WAIT_TIME_MS);
                rxflag = true;
            } else {
                chSysLock();
                decryptDBUS();

#ifdef RC_SAFE_LOCK
                if (lock_state != RC_UNLOCKED)
                    RC_RCreset();
                else if (chVTGetSystemTimeX() > update_time + S2ST(RC_LOCK_TIME_S))
                    lock_state = RC_LOCKED;
#endif

                chSysUnlock();
            }
        } else {
            rxflag = false;
            RC_reset();
            timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
        }

        //Control the flashing of green LED // TODO: Shift to Error.c
        if (!(count % 25)) {
            uint32_t blink_count = count / 25;
            if (!(blink_count % 8))
                LEDB_OFF();
            if (!rxflag ||
                #ifdef RC_SAFE_LOCK
                (lock_state != RC_UNLOCKED && (blink_count % 8 < 2)) ||
                (lock_state == RC_UNLOCKED && (blink_count % 8 < 4))
#else
                (blink_count % 8 < 4)
#endif
                    )
                LEDB_TOGGLE();
        }

        count++;
    }
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void)
{
    RC_reset();

    chThdCreateStatic(uart_dbus_thread_wa, sizeof(uart_dbus_thread_wa),
                      NORMALPRIO + 7,
                      uart_dbus_thread, NULL);
}