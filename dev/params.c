#include "ch.h"
#include "hal.h"

#include "params.h"
#include "flash.h"
#include  <string.h>

/* This is where all the parameters are stored*/
static param_t params[PARAMS_NUM_MAX];
static uint8_t params_count = 0;

/* This is where all the names and characteristics of the parameters are stored*/
static char* params_names[PARAMS_NUM_MAX];
static char* params_subnames[PARAMS_NUM_MAX];
static uint8_t params_per_set[PARAMS_NUM_MAX];

#define RXBUF_SIZE 6
static uint8_t rxbuf[RXBUF_SIZE];
static thread_reference_t uart_receive_thread_handler = NULL;

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp)
{
  switch(rxbuf[0])
  {
    case 'p':
      params[rxbuf[1]] = *((param_t*)(rxbuf+2));
      break;
    case 's':
      break;
    case 'r':
      break;
    case 'c':
      break;
  }

  chSysLockFromISR();
  chThdResumeI(&uart_receive_thread_handler,MSG_OK);
  chSysUnlockFromISR();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,rxend,NULL,NULL,
  115200,
  0,
  0,
  0
};

static THD_WORKING_AREA(params_rx_wa, 128);
static THD_FUNCTION(params_rx,p)
{
  chRegSetThreadName("param receiver");
  (void)p;

  while(!chThdShouldTerminateX())
  {
    uartStartReceive(UART_PARAMS, RXBUF_SIZE, rxbuf);

    chSysLock();
    chThdSuspendS(&uart_receive_thread_handler);
    chSysUnlock();
  }
}

param_t* params_get(void)
{
  return params;
}

param_t* params_set(const uint8_t param_pos, const uint8_t param_num)
{
  uint8_t count = params_count;
  params_count += param_num;
  if(params_count >= PARAMS_NUM_MAX || param_pos + param_num >= PARAMS_NUM_MAX)
    return NULL;

  return (params + count);
}

void params_init(void)
{
  uint8_t i;
  for (i = 0; i < PARAMS_NUM_MAX; i++)
    params[i] = 0.0f;

  uartStart(UART_PARAMS, &uart_cfg);
  chThdCreateStatic(params_rx_wa,sizeof(params_rx_wa),
    NORMALPRIO+7,params_rx,NULL);
}
