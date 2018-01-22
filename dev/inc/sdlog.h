#ifndef _SDLOG_H
#define _SDLOG_H

#define SDLOG_UPDATE_PERIOD_MS  50U
#define SDLOG_NUM_BUFFER  32U

typedef enum{
  SD_NOCARD = 1<<4,
  SD_NOCONNECT = 1<<5,
  SD_LOSE_FRAME = 1<<6
}sdlog_error_t;

typedef struct{
  uint32_t position;
  uint8_t* buf[SDLOG_NUM_BUFFER];
  uint8_t len_buf[SDLOG_NUM_BUFFER];
  sdlog_error_t errorCode;
}sdlogStruct;

void sdlog_init(void);
uint8_t sdlog_createFile(char fileName[]);
uint8_t sdlog_put(const uint8_t pos, const void* const buf, const uint8_t len);

#endif
