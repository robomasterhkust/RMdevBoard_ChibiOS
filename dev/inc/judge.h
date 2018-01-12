/*
 * judge.h
 *
 *  Created on: 2 Jan 2018
 *      Author: Alex Wong
 */

#define JUDGE_USE_2017
//#define JUDGE_USE_2018

#ifndef INC_JUDGE_H_
#define INC_JUDGE_H_

#define SERIAL_JUDGE                &SD6

/*
 * Say no to magic numbers
 */

#define JUDGE_BUFFER_SIZE           SERIAL_BUFFERS_SIZE

#define JUDGE_FRAMEHEAD             165U
#define JUDGE_SIZE_FRAMEHEAD        5U
#define JUDGE_SIZE_CMDID            2U
#define JUDGE_SIZE_FRAMETAIL        2U

#define JUDGE_SHIFT_PACSIZE         ((uint8_t) 1)
#define JUDGE_SHIFT_PACNUMBER       ((uint8_t) 3)
#define JUDGE_SHIFT_PACTYPE         ((uint8_t) 5)
#define JUDGE_SHIFT_DATA            ((uint8_t) 7)

//2017 Specific
#ifdef JUDGE_USE_2017
#define JUDGE_ID_GAMEINFO           ((uint8_t)0x0001)
#define JUDGE_ID_HEALTH_DELTA       ((uint8_t)0x0002)
#define JUDGE_ID_SHOOT              ((uint8_t)0x0003)
#define JUDGE_ID_USER               ((uint8_t)0x0005)

#define JUDGE_DATA_TYPES            3

typedef enum{
  GAMEINFO = 1,
  HLTHINFO,
  PROJECTILEINFO,
};

typedef enum{
  HLTH_DELTA_ARMOUR = 0,
  HLTH_DELTA_BULLET_OVER_SPEED,
  HLTH_DELTA_BULLET_OVER_FREQ,
  HLTH_DELTA_OVER_POWER,
  HLTH_DELTA_MODULE_DETACHED,
  HLTH_DELTA_REG_PUNISH = 0x6,
  HLTH_DELTA_PLATFORM_REFILL = 0xa,
  HLTH_DELTA_ENGINEER_REFILL = 0xb
}health_delta_num_t;

typedef enum {
  ARMOUR_FRONT = 0,
  ARMOUR_LEFT,
  ARMOUR_BACK,
  ARMOUR_RIGHT,
  ARMOUR_UP_ONE,
  ARMOUR_UP_TWO
}armour_num_t;

typedef struct loc_data_t{
  uint8_t   flag;
  float     x;      //Measured in meters
  float     y;
  float     z;
  float     compass;
}__attribute__((packed)) loc_data_t;

typedef struct game_info_t{
  uint32_t      remainTime;
  uint16_t      remainHealth;
  float         feedBackVolt;
  float         feedBackAmp;
  loc_data_t    locData;
  float         pwrBuf;
  //int32_t       volts;
  //uint8_t       lastpacketid;
}__attribute__((packed)) game_info_t;

typedef struct hlth_delta_info_t{
  armour_num_t          hitPos : 4;
  health_delta_num_t    deltaReason : 4;
  uint16_t              deltaVal;
}__attribute__((packed)) hlth_delta_info_t;

/*
typedef struct hlth_delta_info_t{
  armour_num_t          hitPos;
  health_delta_num_t    deltaReason;
  uint16_t              deltaVal;
}__attribute__((packed)) hlth_delta_info_t;
*/
typedef struct projectile_fb_t{
  float     bulletSpeed;
  float     bulletFreq;
  float     golfSpeed;
  float     golfFreq;
}__attribute__((packed)) projectile_fb_t;
#endif

//2018 Specific
#ifdef JUDGE_USE_2018

#define JUDGE_DATA_TYPES            7

typedef enum{
  GAMEINFO = 1,
  HLTHINFO,
  PROJECTILEINFO,
  POWERINFO,
  RFIDINFO,
  ENDGAMEINFO,
  BUFFERINFO
};

typedef enum{
  HLTH_DELTA_ARMOUR = 0,
  HLTH_DELTA_BULLET_OVER_SPEED,
  HLTH_DELTA_BULLET_OVER_FREQ,
  HLTH_DELTA_OVER_POWER,
  HLTH_DELTA_MODULE_DETACHED,
  HLTH_DELTA_REG_PUNISH = 0x6,
  HLTH_DELTA_PLATFORM_REFILL = 0xa,
  HLTH_DELTA_ENGINEER_REFILL = 0xb
}health_delta_num_t;

typedef enum {
  ARMOUR_FRONT = 0,
  ARMOUR_LEFT,
  ARMOUR_BACK,
  ARMOUR_RIGHT,
  ARMOUR_UP_ONE,
  ARMOUR_UP_TWO
}armour_num_t;

typedef struct gps_data_t{
  uint8_t   validflag;
  float     x;      //Measured in meters
  float     y;
  float     z;
  float     yaw;
}__attribute__((packed)) gps_data_t;

typedef struct game_info_t{
  uint32_t      remainTime;
  uint16_t      remainHealth;
  float         feedBackVolt;
  float         feedBackAmp;
  gps_data_t    gpsData;
  float         pwrBuf;
}__attribute__((packed)) game_info_t;

typedef struct hlth_delta_info_t{
  armour_num_t          hitPos : 4;
  health_delta_num_t    deltaReason : 4;
  uint16_t              deltaVal;
}__attribute__((packed)) hlth_delta_info_t;

typedef struct projectile_fb_t{
  float     bulletSpeed;
  float     bulletFreq;
  float     golfSpeed;
  float     golfFreq;
}__attribute__((packed)) projectile_fb_t;

#endif 

void judgedecode(void);

void judgeinit(void);

#endif /* INC_JUDGE_H_ */
