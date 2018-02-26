#include "ch.h"
#include "hal.h"
#include "sdlog.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ff.h"

sdlogStruct logger;
int tim;
char w0[16];

int write_times = 0;

float pi = 3.1415926;
FRESULT writing;

/*
 * Working area for driver.
 */
static uint8_t sd_scratchpad[512];
static FATFS fs;
static FIL fil;
static FRESULT reg_error;


FRESULT er;
char name[10] = "test0.txt";

extern PIMUStruct pIMU;
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

  //fr =  f_mount(&fs, "", 0);
  if(reg_error)
    return (uint8_t)reg_error;

  /* Opens an existing file. If not exist, creates a new file. */
  fr = f_open(&fil, fileName,  FA_CREATE_ALWAYS);//FA_CREAT_ALWAYS-->always creat, overwrite if exist
  if (!fr)//fr == 0 --> the file exists
  {
    fr = f_lseek(&fil, 0); //try to write from beginning
    if (fr)//error occurs when writing
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
  if(logger.buf[pos])
    return -1;
  logger.buf[pos] = buf;
  logger.len_buf[pos] = len;
  logger.position |= (1<<pos);

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



/**
 *  @brief  cast float/int into string and write in
 *  @param  f: a float/int
 *  @param  t: a char indicating the type of f
 *  @param  pos£º the position to be put
 *  @return     the char pointer representing the string
 */

inline char* fitos(float f,char t){
  int s;
  char* c = NULL;
  switch(t){
  case 'f':s = snprintf(NULL,0,"%f",f);
           c = malloc(s+1);
          snprintf(c,s+1,"%f",f);break;
  case 'i':s = snprintf(NULL,0,"%d",(int)f);
           c = malloc(s+1);
           snprintf(c,s+1,"%d",(int)f);break;
  }
  return c;
}

static THD_WORKING_AREA(sdlog_thread_wa,1024);
static THD_FUNCTION(sdlog_thread,p)
{
    uint32_t tick = chVTGetSystemTimeX();

    f_open(&fil, name,  FA_WRITE);
    f_lseek(&fil, 0);
    sdlog_put(1,&pi,4);

    while(true)
    {   pi+=0.1;
      logger.buf[1] = &pi;
      //cast_u u0;
      tick += MS2ST(SDLOG_UPDATE_PERIOD_MS);
      tim = ST2MS(tick);
      //fitob(pi,&u0,1);
      //pi+=0.1;
      //int k;
      int w = 0;
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
        w++;
        error = sdlog_write(i);
        if(error)
        {
          logger.errorCode |= (error << 7);
          break;
        }
        if(!error)
          f_sync(&fil);
      }

      writing = (FRESULT)error;
      if(w>write_times)
        write_times = w;
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
  /*Creat a new log file without overwriting.*/
  int in = 0;
  reg_error = f_mount(&fs, "", 1);

  volatile FRESULT err = f_open(&fil,"test0.txt",FA_OPEN_EXISTING);
  char* mid = malloc(4);
  while(err==FR_OK){
    in++;
    char* head = "test";
    char* tail = ".txt";
    snprintf(mid,4,"%d",in);
    strcpy(name,head);
    strcat(name,mid);
    strcat(name,tail);
    err = f_open(&fil,name,FA_OPEN_EXISTING);
  }
  logger.errorCode |= sdlog_createFile(name);
  free(mid);


  chThdCreateStatic(sdlog_thread_wa, sizeof(sdlog_thread_wa),
                    NORMALPRIO - 10,
                    sdlog_thread, NULL);
}
