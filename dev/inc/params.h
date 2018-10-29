#ifndef _PARAMS_H_
#define _PARAMS_H_

#define PARAMS_VERSION "1.3.01"

//#define PARAMS_USE_UART
#define PARAMS_USE_USB

#if (defined(PARAMS_USE_UART) && defined(PARAMS_USE_USB)) || \
 (!defined(PARAMS_USE_UART) && (!defined(PARAMS_USE_USB)))
  #error "Please specify the comm port for param manager, UART/USB"
#endif

#ifdef PARAMS_USE_UART
  #define UART_PARAMS &UARTD3
  #define PARAMS_BR 115200
#elif defined(PARAMS_USE_USB)
  /*
  You need to add these items to shellcfg.c & Shellcommand commands[]
  #ifdef PARAMS_USE_USB
    {"/xFD",cmd_param_scale},
    {"/xFB",cmd_param_update},
    {"/xFA",cmd_param_tx},
    {"/xF9",cmd_param_rx},
  #endif
  */
  void cmd_param_rx(BaseSequentialStream * chp, int argc, char *argv[]);
  void cmd_param_scale(BaseSequentialStream * chp, int argc, char *argv[]);
  void cmd_param_update(BaseSequentialStream * chp, int argc, char *argv[]);
  void cmd_param_tx(BaseSequentialStream * chp, int argc, char *argv[]);
#endif

#define PARAMS_NUM_MAX        32U
typedef float param_t, *p_param_t;
typedef const char*     param_name_t;

typedef struct{
  param_t kp;
  param_t ki;
  float error_int;
  float error_int_max;
} __attribute__((packed)) pi_controller_t;

typedef struct{
  param_t kp;
  param_t ki;
  param_t kd;
  float error_int;
  float error_int_max;
} __attribute__((packed)) pid_controller_t;

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

extern p_param_t* param_p;
extern void* param_valid;
extern void* param_private;

#endif
