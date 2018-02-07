/**
 * Edward ZHANG, 20180205
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
static int8_t              subparams[PARAMS_NUM_MAX][8];
static uint8_t             params_total = 0;
static uint8_t             subparams_total = 0;
static param_name_t        param_name[PARAMS_NUM_MAX];
static param_name_t        subparam_name[PARAMS_NUM_MAX];

/* for testing*/
p_param_t* param_p = params;
void* param_valid = &param_valid_flag;
void* param_private = &param_private_flag;

#define RXBUF_SIZE 13
static uint8_t rxbuf[RXBUF_SIZE];

#ifdef PARAMS_USE_UART
static thread_reference_t uart_receive_thread_handler = NULL;
static void sendLine(const char* const string)
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

static THD_WORKING_AREA(params_tx_wa, 256);
static THD_FUNCTION(params_tx,p)
{
  chRegSetThreadName("param transmitter");
  (void)p;
  chThdSleepMilliseconds(100);

  char version[] = PARAMS_VERSION;

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &params_total);
  chThdSleepMilliseconds(5);

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &subparams_total);
  chThdSleepMilliseconds(5);

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 4, (uint8_t*)&param_private_flag);
  chThdSleepMilliseconds(10);

  char mode = 1;

  uartStopSend(UART_PARAMS);
  uartStartSend(UART_PARAMS, 1, &mode);  //mode: CHIBIOS USB
  chThdSleepMilliseconds(5);

  sendLine(version);

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
      sendLine(param_name[i]);
      sendLine(subparam_name[i]);
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
    uint8_t param_index, subParam_index;

    switch(rxbuf[0])
    {
      case 0xFD:
        param_index = rxbuf[2] - '0';
        subParam_index = rxbuf[3] - '0';

        if((1<<param_index)&param_valid_flag &&
           subParam_index < subparams[param_index][0])
        {
          subparams[param_index][subParam_index + 1] = (int8_t)(rxbuf[4] - '0');
        }
        break;
      case 0xFB:
        param_save_flash();
        break;
      case 0xFA:
        chSysLockFromISR();
        thread_t* uart_transmit_thread = chThdCreateI(params_tx_wa,sizeof(params_tx_wa),
          NORMALPRIO - 7,params_tx,NULL);
        chThdStartI(uart_transmit_thread);
        chSysUnlockFromISR();
        break;
      case 0xF9:
        param_index = rxbuf[2] - '0';
        subParam_index = rxbuf[3] - '0';

        if((1<<param_index)&param_valid_flag &&
            subParam_index < subparams[param_index][0])
        {
          chSysLockFromISR();
          uint8_t i, byte;
          uint32_t data = 0;
          for (i = 0; i < 8; i++)
          {
            byte = rxbuf[i + 4];
            if(byte >= '0' && byte <= '9')
              byte -= '0';
            else if(byte >= 'a' && byte <= 'f')
              byte -= ('a' - 10);
            data |= ((byte & 0x0f) << (28 - 4*i));
          }

          params[param_index][subParam_index] = *(param_t*)(&data);
          chSysUnlockFromISR();
        }
        break;
        default:
        {/*
          uartStopSend(UART_PARAMS);
          uartStartSend(UART_PARAMS, 3, "wtf");
          */
        }
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
  PARAMS_BR,
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
    uartStopReceive(UART_PARAMS);
    uartStartReceive(UART_PARAMS, RXBUF_SIZE, rxbuf);

    chSysLock();
    chThdSuspendTimeoutS(&uart_receive_thread_handler, MS2ST(1000));
    chSysUnlock();
  }
}
#endif

#ifdef PARAMS_USE_USB
#include "usbcfg.h"
static void sendLine(BaseSequentialStream * chp, const char* const string)
{
  uint8_t i;
  char* byte = string;
  if(string != NULL)
    for (i = 0; i < strlen(string); i++)
      chSequentialStreamPut(chp, *byte++);
  else
    chSequentialStreamPut(chp, '*');

  chThdSleepMilliseconds(20);
  chSequentialStreamPut(chp, '\n');
  chThdSleepMilliseconds(5);
}

void cmd_param_rx(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t* data = argv[0];

  uint8_t index = data[0] - '0';
  uint8_t sub_index = data[1] - '0';

  if((1<<index)&param_valid_flag &&
     sub_index < subparams[index][0])
  {
    chSysLock();
    uint8_t i, byte;
    uint32_t result = 0;
    for (i = 0; i < 8; i++)
    {
      byte = data[i + 2];
      if(byte >= '0' && byte <= '9')
        byte -= '0';
      else if(byte >= 'a' && byte <= 'f')
        byte -= ('a' - 10);
      result |= ((byte & 0x0f) << (28 - 4*i));
    }

    params[index][sub_index] = *(param_t*)(&result);
    chSysUnlock();
  }
}

void cmd_param_scale(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t* data = argv[0];

  uint8_t index = data[0] - '0';
  uint8_t sub_index = data[1] - '0';
  int8_t scale = data[2] - '0';

  if((1<<index)&param_valid_flag &&
     sub_index < subparams[index][0])
    subparams[index][sub_index + 1] = scale;
}

void cmd_param_update(BaseSequentialStream * chp, int argc, char *argv[])
{
  if(!strcmp(argv[0],"update----"))
    param_save_flash();
}

void cmd_param_tx(BaseSequentialStream * chp, int argc, char *argv[])
{
  char version[] = PARAMS_VERSION;

  chThdSleepMilliseconds(100);

  chSequentialStreamPut(chp, params_total);
  chThdSleepMilliseconds(10);

  chSequentialStreamPut(chp, subparams_total);
  chThdSleepMilliseconds(10);

  char* byte = (char*)&param_private_flag;
  uint8_t i,j;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);
  chThdSleepMilliseconds(10);

  chSequentialStreamPut(chp, 0); //mode: CHIBIOS UART
  chThdSleepMilliseconds(10);

  sendLine(chp, version);
  uint32_t flag;

  flag = 1;
  for (i = 0; i < PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      byte = (char*)subparams[i];
      for (j = 0; j < 8; j++)
        chSequentialStreamPut(chp, *byte++);
      chThdSleepMilliseconds(10);

      chSequentialStreamPut(chp, i);
      chThdSleepMilliseconds(10);
    }
    flag = flag<<1;
  }

  flag = 1;
  for(i = 0; i<PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      byte = (char*)params[i];
      for (j = 0; j < 4*subparams[i][0]; j++)
        chSequentialStreamPut(chp, *byte++);
      chThdSleepMilliseconds(10);
    }
    flag = flag<<1;
  }

  flag = 1;
  for(i = 0; i<PARAMS_NUM_MAX; i++)
  {
    if(flag&param_valid_flag)
    {
      sendLine(chp, param_name[i]);
      sendLine(chp, subparam_name[i]);
    }
    flag = flag<<1;
  }
}
#endif

static uint8_t param_load_flash(const uint8_t param_pos, const uint8_t param_num)
{
  uint8_t result = 0;

  flashaddr_t address = PARAM_FLASH_ADDR + 16 + param_pos*PARAM_FLASH_BLOCK;
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
      result = 2;
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
  uint8_t result = 0;

  //Maximum num of parameters and subparameters supported
  if(param_num > 7 || param_pos >= PARAMS_NUM_MAX)
    return 1;

  //Check whether this position is occupied or not
  if(param_valid_flag & ((uint32_t)1<<param_pos))
    return 2;
  else
    param_valid_flag |= ((uint32_t)1<<param_pos);

  params[param_pos] = p_param;
  subparams[param_pos][0] = param_num;

  //Read out raw data on flash, and check validity
  if(param_load_flash(param_pos, param_num))
    result = 3;

  param_name[param_pos] = Param_name;
  subparam_name[param_pos] = subParam_name;

  if(param_private == PARAM_PRIVATE)
    param_private_flag |= ((uint32_t)1 << param_pos);

  params_total++;
  subparams_total += param_num;

  return result;
}

void param_save_flash(void)
{
  LEDR_ON();
  LEDG_OFF();

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

  LEDR_OFF();
}

void params_init(void)
{
  param_valid_flag = 0;
  param_private_flag = 0;

  #ifdef PARAMS_USE_UART
    uartStart(UART_PARAMS, &uart_cfg);
    chThdCreateStatic(params_rx_wa,sizeof(params_rx_wa),
      NORMALPRIO + 7,params_rx,NULL);
  #endif
}
