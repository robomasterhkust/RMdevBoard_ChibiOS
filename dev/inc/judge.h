/*
 * judge.h
 *
 *  Created on: 2 Jan 2018
 *      Author: Alex Wong
 */

#ifndef INC_JUDGE_H_
#define INC_JUDGE_H_

//#define JUDGE_USE_2017

#ifndef JUDGE_USE_2017
#define JUDGE_USE_2018
#endif

#define SERIAL_JUDGE                &SD6

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

//2018 Specific
#ifdef JUDGE_USE_2018

#define JUDGE_DATA_TYPES            8

typedef enum{
  GAMEINFO = 1,
  HLTHINFO,
  PROJECTILEINFO,
  POWERINFO,
  RFIDINFO,
  ENDGAMEINFO,
  BUFFERINFO,
  POSITIONINFO,
  USERDATA = 0x0100
};

typedef struct game_info_t{
  uint16_t      remainTime;
  uint8_t       gameStatus;
  uint8_t       robotLevel;
  uint16_t      remainHealth;
  uint16_t      fullHealth;
}__attribute__((packed)) game_fb_t;

typedef struct hlth_info_t{
  uint8_t       hitPos : 4;
  uint8_t       deltaReason : 4;
}__attribute__((packed)) hlth_fb_t;

typedef struct projectile_fb_t{
  uint8_t       bulletType;
  uint8_t       bulletFreq;
  float         bulletSpeed;
}__attribute__((packed)) projectile_fb_t;

typedef struct power_fb_t{
  float         volt;
  float         current;
  float         power;
  float         powerBuffer;
  uint16_t      shooterHeat0;
  uint16_t      shooterHeat1;
}__attribute__((packed)) power_fb_t;

typedef struct rfid_fb_t{
  uint8_t       cardType;
  uint8_t       cardIdx;
}__attribute__((packed)) rfid_fb_t;

typedef struct game_over_fb_t{
  uint8_t       winner;
}__attribute__((packed)) game_over_fb_t;

typedef struct buffer_fb_t{
  uint8_t       powerUpType;
  uint8_t       powerUpPercentage;
}__attribute__((packed)) buffer_fb_t;

typedef struct location_fb_t{
  float         x;
  float         y;
  float         z;
  float         yaw;
}__attribute__((packed)) location_fb_t;

typedef struct judge_fb_t{
  game_fb_t         gameInfo;
  hlth_fb_t         hlthInfo;
  projectile_fb_t   projectileInfo;
  power_fb_t        powerInfo;
  rfid_fb_t         rfidInfo;
  game_over_fb_t    gameOverInfo;
  buffer_fb_t       bufferInfo;
  location_fb_t     locationInfo;
}__attribute__((packed)) judge_fb_t;

typedef struct user_data_t{
  float         data1;
  float         data2;
  float         data3;
  uint8_t       mask;
}__attribute__((packed)) user_data_t;

#endif 

judge_fb_t judgeDataGet(void);

void judgedecode(void);

void judgeinit(void);

#endif /* INC_JUDGE_H_ */

