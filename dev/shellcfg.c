/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */
#include "main.h"
#include "shell.h"

/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_host
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#include <math.h>
#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(Host_thread_wa, 512);
static THD_FUNCTION(Host_thread, p)
{
  (void)p;
  chRegSetThreadName("Host tramsmitter");

  if((*SERIAL_DATA).state != SD_READY)
    sdStart(SERIAL_DATA, NULL);

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)SERIAL_DATA;

  uint32_t tick = chVTGetSystemTimeX();

  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (true)
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    txbuf_d[0] = (int16_t)(tick % 10000);
    txbuf_f[0] = sinf((float)(txbuf_d[0] - 5000) * M_PI/5000.0f);

    transmit_host(chp, txbuf_d, txbuf_f, 1, 1);
  }
}

/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  GimbalStruct* gimbal = gimbal_get();

  chprintf(chp,"Pitch angle: %f\r\n",gimbal->pitch_angle);
  chprintf(chp,"Yaw angle: %f\r\n",gimbal->yaw_angle);
  chprintf(chp,"Pitch current: %f\r\n",gimbal->pitch_current);
  chprintf(chp,"Yaw current: %f\r\n",gimbal->yaw_current);
  chprintf(chp,"Pitch current sp: %d\r\n",gimbal->encoder_can[GIMBAL_PITCH].current_setpoint);
  chprintf(chp,"Yaw current sp: %d\r\n",gimbal->encoder_can[GIMBAL_YAW].current_setpoint);
}

/**
 * @brief Start the data tramsmission to host machine
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);
  }

  chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
  chThdSleepSeconds(sec);

  chThdCreateStatic(Host_thread_wa, sizeof(Host_thread_wa),
  NORMALPRIO,
                    Host_thread, NULL);
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"data", cmd_data}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)SERIAL_CMD,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  sdStart(SERIAL_CMD, NULL);

  shellInit();
  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);
}
