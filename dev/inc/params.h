#ifndef _PARAMS_H_
#define _PARAMS_H_

#define UART_PARAMS &UARTD2
#define PARAMS_NUM_MAX 30U

typedef float param_t;

param_t* params_set(const uint8_t param_pos, const uint8_t param_num);
param_t* params_get(void);
void params_init(void);

#endif
