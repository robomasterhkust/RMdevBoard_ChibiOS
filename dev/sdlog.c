#include "ch.h"
#include "hal.h"
#include "sdlog.h"

#include "ff.h"

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

static THD_WORKING_AREA(sdlog_thread_wa,256);
static THD_FUNCTION(sdlog_thread,p)
{
  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += MS2ST(SDLOG_UPDATE_PERIOD_MS);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }
  }
}

void sdlog_init(void)
{
  sdcStart(&SDCD1,&sdccfg);

  /* Card presence check.*/
  if (!blkIsInserted(&SDCD1))
  {
    return;
  }

  if (sdcConnect(&SDCD1))
  {
    return;
  }

  uint8_t result = sdlog_createFile("test.txt");

  chThdCreateStatic(sdlog_thread_wa, sizeof(sdlog_thread_wa),
                    NORMALPRIO - 10,
                    sdlog_thread, NULL);
}
