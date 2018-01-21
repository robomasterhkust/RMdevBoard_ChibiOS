#ifndef _SDLOG_H
#define _SDLOG_H

#define SDLOG_UPDATE_PERIOD_MS  1000U

void sdlog_init(void);
uint8_t sdlog_createFile(char fileName[]);

#endif
