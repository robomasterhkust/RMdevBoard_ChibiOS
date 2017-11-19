#ifndef _PARAMS_H_
#define _PARAMS_H_

#define UART_PARAMS &UARTD3
#define PARAMS_NUM_MAX        32U

typedef float param_t, *p_param_t;
typedef const char*     param_name_t;

typedef enum {
  PARAM_PUBLIC = 0,
  PARAM_PRIVATE = 1
} param_public_flag_t;

static const char subname_PI[] = "KP KI";
static const char subname_PID[] = "KP KI KD";

uint8_t params_set(param_t* const     p_param,
                  const uint8_t       param_pos,
                  const uint8_t       param_num,
                  param_name_t const  Param_name,
                  param_name_t const  subParam_name,
                  param_public_flag_t param_private);
void params_init(void);
void param_save_flash(void);

#endif
