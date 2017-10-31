#include "ch.h"
#include "hal.h"

#include "dbus.h"
//#include "chprintf.h"
//static BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_CMD;

static uint8_t rxbuf[DBUS_BUFFER_SIZE];
static RC_Ctl_t RC_Ctl;
static thread_reference_t uart_host_thread_handler = NULL;
static uint8_t rx_start_flag = 1;

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
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {

  if(rx_start_flag)
  {
    chSysLockFromISR();
    chThdResumeI(&uart_host_thread_handler, MSG_OK);
    chSysUnlockFromISR();
  }
  else
    rx_start_flag = 1;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,rxend,NULL,NULL,
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
  bool rxflag = false;
  systime_t timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
  uint32_t count = 0;

  while(!chThdShouldTerminateX())
  {
    uartStopReceive(UART_DBUS);
    uartStartReceive(UART_DBUS, DBUS_BUFFER_SIZE, rxbuf);

    chSysLock();
    rxmsg = chThdSuspendTimeoutS(&uart_host_thread_handler, timeout);
    chSysUnlock();

    if(rxmsg == MSG_OK)
    {
      if(!rxflag)
      {
        timeout = MS2ST(DBUS_WAIT_TIME_MS);
        rxflag = true;
      }
      else
        decryptDBUS();
    }
    else
    {
      rxflag = false;
      timeout = MS2ST(DBUS_INIT_WAIT_TIME_MS);
    }

    //Control the flashing of green LED
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

static void rcStructInit(void)
{
  RC_Ctl.rc.channel0 = 1024;
  RC_Ctl.rc.channel1 = 1024;
  RC_Ctl.rc.channel2 = 1024;
  RC_Ctl.rc.channel3 = 1024;
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
