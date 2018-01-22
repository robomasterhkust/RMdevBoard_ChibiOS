#include "ch.h"
#include "hal.h"
#include "sdlog.h"

#include "ff.h"

sdlogStruct logger;

/*
 * Working area for driver.
 */
static uint8_t sd_scratchpad[512];
static FATFS fs;
static FIL fil;

/*
 * SDIO configuration.
 */
static const SDCConfig sdccfg = {
  sd_scratchpad,
  SDC_MODE_4BIT
};

/**
 *   @brief   return the error code of SD log
 *   @api
 */
uint32_t sdlog_getError(void)
{
  return (uint32_t)logger.errorCode;
}

/**
 *   @brief   create a new file on SD card
 *   @NOTE    will overwrite file with same name
 *   @notapi
 */
uint8_t sdlog_createFile(char fileName[])
{
  FRESULT fr;

  fr =  f_mount(&fs, "", 1);
  if(fr)
    return (uint8_t)fr;

  /* Opens an existing file. If not exist, creates a new file. */
  fr = f_open(&fil, fileName,  FA_CREATE_ALWAYS);
  if (!fr)
  {
    fr = f_lseek(&fil, 0);
    if (fr)
    {
      f_close(&fil);
      return fr;
    }
  }

  f_close(&fil);
  return (uint8_t)fr;
}

/**
 *    @brief  indicate the data to be logged
 *    @param  pos: position of the data to be logged [0-32]
 *    @param  buf: data array to be logged
 *    @param  len: length of data array in byte
 *    @NOTE   return -1 if that paticular position is occupied
 *    @api
 */
uint8_t sdlog_put(const uint8_t pos, const void* const buf, const uint8_t len)
{
  //TODO put the buf* and length to sdlogstruct and verify it
  return 0;
}

/**
 *  @brief  write log file with the defined format
 *  @return the error code while writing, 0 if suceeded
 *  @notapi
 */
static inline uint8_t sdlog_write(const uint8_t pos)
{
  uint8_t len;
  FRESULT fr;
  fr = f_write(&fil, logger.buf[pos], logger.len_buf[pos], &len);

  return (uint8_t)fr;
}

static THD_WORKING_AREA(sdlog_thread_wa,1024);
static THD_FUNCTION(sdlog_thread,p)
{
  uint32_t tick = chVTGetSystemTimeX();

  /* for test*/
  char buf1[] = "   \n";
  char buf2[] = "12345678\n12345678\n";
  char buf3[] = "abcdefgh\n12345678\n";
  char buf4[] = "abcdefgh\nabcdefgh\n";
  logger.buf[0] = buf1;
  logger.buf[1] = buf2;
  logger.buf[2] = buf3;
  logger.buf[3] = buf4;
  logger.len_buf[0] = 4;
  logger.len_buf[1] = 18;
  logger.len_buf[2] = 18;
  logger.len_buf[3] = 18;
  logger.position = 0x0006;
  /**/

  f_open(&fil, "test.txt",  FA_WRITE);
  f_lseek(&fil, 0);

  while(true)
  {
    tick += MS2ST(SDLOG_UPDATE_PERIOD_MS);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      logger.errorCode |= SD_LOSE_FRAME;
    }

    uint8_t i, error;
    for(i = 0; i<SDLOG_NUM_BUFFER; i++)
    {
      if(!(logger.position & (1<<i)))
        continue;
      error = sdlog_write(i);
      if(error)
      {
        logger.errorCode |= (error << 7);
        break;
      }
    }

    if(!error)
      f_sync(&fil);
  }
}

/**
 *   @brief   initializes SD logger
 *   @api
 */
void sdlog_init(void)
{
  memset((void*)&logger, 0, sizeof(sdlogStruct));
  sdcStart(&SDCD1,&sdccfg);

  /* Card presence check.*/
  if (!blkIsInserted(&SDCD1))
  {
    logger.errorCode |= SD_NOCARD;
    return;
  }

  if (sdcConnect(&SDCD1))
  {
    logger.errorCode |= SD_NOCONNECT;
    return;
  }

  logger.errorCode |= sdlog_createFile("test.txt");

  chThdCreateStatic(sdlog_thread_wa, sizeof(sdlog_thread_wa),
                    NORMALPRIO - 10,
                    sdlog_thread, NULL);
}
