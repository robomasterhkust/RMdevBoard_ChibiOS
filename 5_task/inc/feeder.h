#ifndef FEEDER
#define FEEDER

#define OUTPUT_MAX 6000
#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_STOP 3
#define FEEDER_SINGLE 1
#define FEEDER_LONG 2

#define NORMAL_TURN 0
#define ERROR_TURN  1

//int16_t return_measured(int16_t *);

typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;

#ifdef __cplusplus
extern "C" {
#endif

void feeder_bulletOut(void);
void feeder_singleShot(void);
void feeder_func(int mode);
void feeder_init(void);

#ifdef __cplusplus
}
#endif

#endif
