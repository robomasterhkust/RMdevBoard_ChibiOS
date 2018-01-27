#include "ch.h"
#include "hal.h"
#include "sdlog.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ff.h"
sdlogStruct logger;
int tim;
char w0[10];
char w1[10];
char w2[10];
char w3[10];

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
  //TODO put the buf* and length to sdlogstruct and verify it
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

  //fr = f_write(&fil, logger.buf[pos], logger.len_buf[pos], &len);
  if(pos==1)
    fr = f_write(&fil,(int *)logger.buf[pos], logger.len_buf[pos],&len);
  else if(pos%2)
    fr = f_write(&fil,(float *)logger.buf[pos], logger.len_buf[pos],&len);
  else
    fr = f_write(&fil,(char *)logger.buf[pos], logger.len_buf[pos],&len);

  return (uint8_t)fr;
}

static THD_WORKING_AREA(sdlog_thread_wa,1024);
static THD_FUNCTION(sdlog_thread,p)
{
    uint32_t tick = chVTGetSystemTimeX();
    /* testing writing in IMU acceleration data log*/
    logger.position = 0x001E; //use buf[1]-buf[4]
    /* configuring file head*/
    int k;
    logger.len_buf[1] = 5;
    for(k=2;k<8;k++){
      if(k%2)
        logger.len_buf[k] = 8;
      else{
        logger.len_buf[k] = 1;
        logger.buf[k] = "\t";
      }
    }
    logger.len_buf[8] = 1;
    logger.buf[8] = "\n";
    /*char buff1[20] = "type:";
    char buff2[20] = "1systime";
    char buff3[20] = "3float";
    char buff4[20] = "\n";
    logger.buf[1] = buff1;
    logger.buf[2] = buff2;
    logger.buf[3] = buff3;
    logger.buf[4] = buff4;*/

    f_open(&fil, name,  FA_WRITE);
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

      tim = ST2MS(tick);
      /*char s0[10];
      char b0[10];
      char b1[10];
      char b2[10];
      /*int s_i = snprintf(NULL,0,"%d",s0);
      int s_f = snprintf(NULL,0,"%d",b0);**/
      /*snprintf(s0,5,"%d",tim);
      snprintf(b0,8,"%f",pIMU->accelData[0]);
      snprintf(b1,8,"%f",pIMU->accelData[1]);
      snprintf(b2,8,"%f",pIMU->accelData[2]);
      //strcat(b2,re);
      /*strcpy(logger.buf[1],s0);
      strcpy(logger.buf[3],b0);
      strcpy(logger.buf[5],b1);
      strcpy(logger.buf[7],b2);*/
      /*strcat(s0," ");
      strcat(b0," ");
      strcat(b1," ");
      strcat(b2,"\n");*/

      /*logger.len_buf[1] = s_i+1;
      logger.len_buf[2] = s_f+1;
      logger.len_buf[3] = s_f+1;
      logger.len_buf[4] = s_f+1;*/
      logger.buf[1] = &tim;
      logger.buf[3] = &pIMU->accelData[0];
      logger.buf[5] = &pIMU->accelData[1];
      logger.buf[7] = &pIMU->accelData[2];
      strcpy(w0,logger.buf[1]);
      strcpy(w1,logger.buf[3]);
      strcpy(w2,logger.buf[5]);
      strcpy(w3,logger.buf[7]);
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
  int in = 0;
  reg_error = f_mount(&fs, "", 1);

  volatile FRESULT err = f_open(&fil,"test0.txt",FA_OPEN_EXISTING);
  while(err==FR_OK){
    in++;
    char* head = "test";
    char* tail = ".txt";
    char* mid = malloc(4);
    snprintf(mid,4,"%d",in);
    strcpy(name,head);
    strcat(name,mid);
    strcat(name,tail);
    err = f_open(&fil,name,FA_OPEN_EXISTING);
  }

  logger.errorCode |= sdlog_createFile(name);

  chThdCreateStatic(sdlog_thread_wa, sizeof(sdlog_thread_wa),
                    NORMALPRIO - 10,
                    sdlog_thread, NULL);
}
