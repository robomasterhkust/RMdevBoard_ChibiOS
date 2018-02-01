/**
 * Edward ZHANG, 20171111
 * @file    param.c
 * @brief   parameter tuning and management interface
 */

#include "ch.h"
#include "hal.h"

#include "params.h"
#include "flash.h"
#include  <string.h>

#define PARAM_FLASH_SECTOR           11U
#define PARAM_FLASH_ADDR      0x080E0000

#define PARAM_FLASH_HALF_BLOCK       64U
#define PARAM_FLASH_BLOCK           128U

static uint32_t            param_valid_flag = 0;
static uint32_t            param_private_flag = 0;
static p_param_t           params[PARAMS_NUM_MAX];
static uint8_t             subparams[PARAMS_NUM_MAX][8];
static uint8_t             params_total = 0;
static uint8_t             subparams_total = 0;
static param_name_t        param_name[PARAMS_NUM_MAX];
static param_name_t        subparam_name[PARAMS_NUM_MAX];

#define RXBUF_SIZE 7
static uint8_t rxbuf[RXBUF_SIZE];
static thread_reference_t uart_receive_thread_handler = NULL;

static void uart_sendLine(const char* const string)
{
  char terminator = '\n';
  uartStopSend(UART_PARAMS);
  if(string != NULL)
    uartStartSend(UART_PARAMS, strlen(string), string);
  else
  {
    char null_char = '*';
    uartStartSend(UART_PARAMS, 1, &null_char);
  }
  chThdSleepMilliseconds(20);
  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &terminator);
  chThdSleepMilliseconds(5);
}

static THD_WORKING_AREA(params_tx_wa, 128);
static THD_FUNCTION(params_tx,p)
{
  chRegSetThreadName("param transmitter");
  (void)p;
  chThdSleepMilliseconds(10);

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &params_total);
  chThdSleepMilliseconds(5);

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &subparams_total);
  chThdSleepMilliseconds(5);

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 4, (uint8_t*)&param_private_flag);
  chThdSleepMilliseconds(5);

  uint8_t i;
  uint32_t flag;

  flag = 1;
  for (i = 0; i < PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      uartStopSend(UART_PARAMS);
      uartStartSend(UART_PARAMS, 8, subparams[i]);
      chThdSleepMilliseconds(10);

      uartStopSend(UART_PARAMS);
      uartStartSend(UART_PARAMS, 1, &i);
      chThdSleepMilliseconds(5);
    }
    flag = flag<<1;
  }

  flag = 1;
  for(i = 0; i<PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      uartStopSend(UART_PARAMS);
      uartStartSend(UART_PARAMS, 4*subparams[i][0], (uint8_t*)(params[i]));
      chThdSleepMilliseconds(10);
    }
    flag = flag<<1;
  }

  flag = 1;
  for(i = 0; i<PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      uart_sendLine(param_name[i]);
      uart_sendLine(subparam_name[i]);
    }
    flag = flag<<1;
  }

  chThdExit(MSG_OK);
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp)
{
  if(uartp == UART_PARAMS)
  {
    switch(rxbuf[0])
    {
      case 'p':
        if((1<<rxbuf[1])&param_valid_flag &&
           rxbuf[2] < subparams[rxbuf[1]][0])
        {
          chSysLockFromISR();
          params[rxbuf[1]][rxbuf[2]] = *((param_t*)(rxbuf+3));
          chSysUnlockFromISR();
        }
        break;
      case 's':
        if((1<<rxbuf[1])&param_valid_flag &&
           rxbuf[2] < subparams[rxbuf[1]][0])
          subparams[rxbuf[1]][rxbuf[2] + 1] = rxbuf[3];
        break;
      case 'u':
        param_save_flash();
        break;
      case 'g':
        chSysLockFromISR();
        thread_t* uart_transmit_thread = chThdCreateI(params_tx_wa,sizeof(params_tx_wa),
          NORMALPRIO - 7,params_tx,NULL);
        chThdStartI(uart_transmit_thread);
        chSysUnlockFromISR();
        break;
    }

    chSysLockFromISR();
    chThdResumeI(&uart_receive_thread_handler,MSG_OK);
    chSysUnlockFromISR();
  }
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

static uint8_t param_load_flash(const uint8_t param_pos, const uint8_t param_num)
{
  uint8_t result = 0;

  flashaddr_t address = PARAM_FLASH_ADDR + 16 +
                        param_pos*PARAM_FLASH_BLOCK;
  flashRead(address,subparams[param_pos],8);

  uint8_t i;
  if(subparams[param_pos][0] != param_num)
  {
    for(i = 1; i < 8; i++)
      subparams[param_pos][i] = 0;
    subparams[param_pos][0] = param_num;
    result = 1;
  }

  flashRead(address + PARAM_FLASH_HALF_BLOCK,
            (char*)(params[param_pos]), subparams[param_pos][0]*4);

  for (i = 0; i < param_num; i++)
    //Check the validity of the read-out value
    if(params[param_pos][i] != params[param_pos][i] ||
       params[param_pos][i] > 1.701411e38 ||
       params[param_pos][i] < -1.701411e38)
    {
      params[param_pos][i] = 0.0f;
      result = 1;
    }
  return result;
}

uint8_t params_set(param_t* const     p_param,
                  const uint8_t       param_pos,
                  const uint8_t       param_num,
                  param_name_t const  Param_name,
                  param_name_t const  subParam_name,
                  param_public_flag_t param_private)
{
  //Maximum num of parameters and subparameters supported
  if(param_num > 7 || param_pos >= PARAMS_NUM_MAX)
    return 1;

  uint8_t result = 0;
  //Check whether this position is occupied or not
  if(param_valid_flag & (1<<param_pos))
    return 2;
  else
    param_valid_flag |= (uint32_t)(1<<param_pos);

  params[param_pos] = p_param;
  subparams[param_pos][0] = param_num;

  if(param_load_flash(param_pos, param_num))
    result = 3;

  param_name[param_pos] = Param_name;
  subparam_name[param_pos] = subParam_name;

  if(param_private == PARAM_PRIVATE)
    param_private_flag |= (uint32_t)(1 << param_pos);

  params_total++;
  subparams_total += param_num;

  return result;
}

void params_init(void)
{
  param_valid_flag = 0;
  param_private_flag = 0;

  uartStart(UART_PARAMS, &uart_cfg);
  chThdCreateStatic(params_rx_wa,sizeof(params_rx_wa),
    NORMALPRIO + 7,params_rx,NULL);
}

void param_save_flash(void)
{
  flashSectorErase(PARAM_FLASH_SECTOR);
  uint8_t i;
  uint32_t flag = 1;
  flashaddr_t address;

  for(i = 0; i< PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      address = PARAM_FLASH_ADDR + 16 + PARAM_FLASH_BLOCK*i;
      flashWrite(address,subparams[i],8);
      flashWrite(address + PARAM_FLASH_HALF_BLOCK,
        (char*)(params[i]), subparams[i][0]*4);
    }
    flag = flag<<1;
  }
}
