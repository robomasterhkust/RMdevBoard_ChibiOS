#ifndef FEEDER
#define FEEDER

#define FEEDER_SINGLE_TIMEOUT_MS 100U

#define FEEDER_CAN &CAND1
#define FEEDER_CAN_EID 0x1FF

#define FEEDER_LS_GPIO          GPIOB
#define FEEDER_LS_PIN_NO   GPIOB_PIN0
#define FEEDER_LS_PIN_NC   GPIOB_PIN1

#define FEEDER_BULLET_PER_TURN  7U
#define FEEDER_GEAR             36U
#define FEEDER_SET_RPS          19U     //Rounds per second of feeder

#define FEEDER_OUTPUT_MAX       16383U
#define FEEDER_OUTPUT_MAX_BACK  12000U  //output limit for stuck-bullet turning back
#define FEEDER_ERROR_COUNT        800U

#define FEEDER_MINIGUN_RPS        18U

#define FEEDER_BOOST_POWER  16383U

typedef enum{
  FEEDER_STOP = 0,
  FEEDER_SINGLE,    //Single shot
  FEEDER_AUTO,      //Auto fire
  FEEDER_FINISHED,  //Finished a round of shooting
  FEEDER_OVERHEAT,  //OVER HEAT!
  FEEDER_BOOST      //FEEDER_USE_BOOST
}feeder_mode_t;

typedef enum{
  FEEDER_OK = 0,
  FEEDER_CONNECTION_ERROR = 1,    //Motor connection error
  LIMIT_SWITCH_ERROR_0 = 2,       //Limit switch connection error
  LIMIT_SWITCH_ERROR_1 = 4,       //Limit switch falloff
  LIMIT_SWITCH_ERROR_2 = 8,       //Limit switch error
  LIMIT_SWITCH_ERROR = 14
}feeder_error_t;

typedef struct{
    float kp;
    float ki;
    float kd;
    float inte_max;
    float inte;
} __attribute__((packed)) pid_struct;

int16_t feeder_canUpdate(void); //In case the feeder ESC is using the same EID as gimbal does
feeder_error_t feeder_get_error(void);
float feeder_getDelay(void);
void feeder_bulletOut(void);  //Used as limit switch EXTI funtion
void feeder_singleShot(void); //Rune shooting function

void feeder_init(void);
void feeder_start(void);
int16_t feeder_getSpeed(void);

feeder_mode_t* get_feeder(void);
uint8_t* get_feeder_fire_mode(void);

#endif
