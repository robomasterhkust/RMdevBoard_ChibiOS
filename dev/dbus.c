/**
 * Edward ZHANG, Terry ZENG, 201709??
 * @file    dbus.c
 * @brief   Dbus driver and decoder
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
typedef bool dbus_error_t;
static dbus_error_t rxflag = false;

static void rcStructInit(void)
{
    RC_Ctl.rc.channel0 = 1024;
    RC_Ctl.rc.channel1 = 1024;
    RC_Ctl.rc.channel2 = 1024;
    RC_Ctl.rc.channel3 = 1024;
    RC_Ctl.rc.s1 =0;
    RC_Ctl.rc.s2 = 0;
    RC_Ctl.mouse.LEFT=0;
    RC_Ctl.mouse.RIGHT =0;
    RC_Ctl.mouse.x=0;
    RC_Ctl.mouse.y=0;
    RC_Ctl.mouse.z=0;
    RC_Ctl.keyboard.key_code=0;
}

/*
 * @brief get error code: rx_flag
 * @status: true = connected
 *          false = not connected
 */
dbus_error_t dbus_getError(void){
  return rxflag;
}

/**
 * @brief   Decode the received DBUS sequence and store it in RC_Ctl struct
 */
static void decryptDBUS(void)
{
  RC_Ctl.rc.channel0 = ((rxbuf[0]) | (rxbuf[1]<<8)) & 0x07FF;
  RC_Ctl.rc.channel1 = ((rxbuf[1]>>3) | (rxbuf[2]<<5)) & 0x07FF;
  RC_Ctl.rc.channel2 = ((rxbuf[2]>>6) | (rxbuf[3]<<2) | ((uint32_t)rxbuf[4]<<10)) & 0x07FF;
  RC_Ctl.rc.channel3 = ((rxbuf[4]>>1) | (rxbuf[5]<<7)) & 0x07FF;
  RC_Ctl.rc.s1  = ((rxbuf[5] >> 4)& 0x000C) >> 2;                         //!< Switch left
  RC_Ctl.rc.s2  = ((rxbuf[5] >> 4)& 0x0003);


  RC_Ctl.mouse.x = rxbuf[6] | (rxbuf[7] << 8);                   //!< Mouse X axis
  RC_Ctl.mouse.y = rxbuf[8] | (rxbuf[9] << 8);                   //!< Mouse Y axis
  RC_Ctl.mouse.z = rxbuf[10] | (rxbuf[11] << 8);                 //!< Mouse Z axis
  RC_Ctl.mouse.LEFT = rxbuf[12];                                       //!< Mouse Left Is Press ?
  RC_Ctl.mouse.RIGHT = rxbuf[13];                                       //!< Mouse Right Is Press ?
  RC_Ctl.keyboard.key_code = rxbuf[14] | (rxbuf[15] << 8);                   //!< KeyBoard value
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  if (rx_start_flag) {
    chSysLockFromISR();
    chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
    chSysUnlockFromISR();
  } else
    rx_start_flag = 1;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
        NULL, NULL, rxend, NULL, NULL,
        100000,
        USART_CR1_PCE,
        0,
        0
};

#define  DBUS_INIT_WAIT_TIME_MS      4U
#define  DBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_dbus_thread_wa, 512);
static THD_FUNCTION(uart_dbus_thread, p)
{
  (void)p;
  chRegSetThreadName("uart dbus receiver");

  uartStart(UART_DBUS, &uart_cfg);
  dmaStreamRelease(*UART_DBUS.dmatx);

  size_t rx_size;
  msg_t rxmsg;
  rxflag = false;
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
      } else
        decryptDBUS();
    } else {
      rxflag = false;
      rcStructInit();
      timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
    }

    //Control the flashing of green LED // Shift to Error.c
    if((!(count % 25) && !rxflag) || !(count% 75))
      LEDG_TOGGLE();
    count++;

  }
}

/**
 * @brief   Return the RC_Ctl struct
 */
RC_Ctl_t* RC_get(void)
{
  return &RC_Ctl;
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void)
{
  rcStructInit();

  chThdCreateStatic(uart_dbus_thread_wa, sizeof(uart_dbus_thread_wa),
                    NORMALPRIO + 7,
                    uart_dbus_thread, NULL);
}
