/*
 * adis16265.h
 *
 *  Created on: 19 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_ADIS16265_H_
#define INC_ADIS16265_H_

#include "params.h"
#include "math_misc.h"

//Type of Gyro
#ifndef GYRO
  #define GYRO
  #define GYRO_ADIS
  #define GYRO_INTERFACE_SPI
#else
  #error "Multiple gyro config file found, stop"
#endif

typedef enum{
  GYRO_PSC_320 = 0,
  GYRO_PSC_160,
  GYRO_PSC_80
} gyro_psc_conf_t;

/*==================USER CONFIGURATION==================*/
/* Update frequency of Gyro in Hz */
#define GYRO_UPDATE_FREQ        500U

/* User SPI interface configuration */
#define GYRO_SPI                &SPID4
#define GYRO_PSC_CONF           GYRO_PSC_320 // setting the Dynamic Range 320/sec

#define GYRO_MOSI               GPIOE_SPI4_ADIS_MOSI
#define GYRO_MISO               GPIOE_SPI4_ADIS_MISO
#define GYRO_CLK                GPIOE_SPI4_ADIS_SCK

#define GYRO_NSS_pin            GPIOE_SPI4_ADIS_NSS
#define GYRO_NSS_port           GPIOE

/* Sensor specific configuration*/
//#define GYRO_RESET_PIN          GPIOA_PIN15
//#define GYRO_RESET_PORT         GPIOA

/*==========END OF USER CONFIGURATION==================*/

#define GYRO_ANG_VEL_TH         0.0080f
#define GYRO_PSC                0.00127863f

#define GYRO_CAL_FLASH          0x08012000

#define GYRO_FLASH              0x01
#define GYRO_POWER              0x03
#define GYRO_VEL                0x05
#define GYRO_ADC                0x0B
#define GYRO_TEMP               0x0D
#define GYRO_ANGL               0x0F
#define GYRO_OFF                0x15
#define GYRO_SCALE              0x17
#define GYRO_COMD               0x3F
#define GYRO_SENS               0x39
#define GYRO_SMPL               0x37

#define GYRO_UPDATE_PERIOD_US 1000000U/GYRO_UPDATE_FREQ

#define X 0
#define Y 1

typedef uint16_t gyrodata_t;

typedef enum{
  NOT_INITED = 0,
  INITED = 1,
  CALIBRATING = 2
} gyro_state_t;

typedef enum {
  GYRO_OK = 0,
  GYRO_CORRUPTED_Q_DATA = 1<<1,
  GYRO_LOSE_FRAME = 1<<7
} gyro_error_t;


typedef struct {
  gyro_state_t state;
  uint8_t error_flag;
  SPIDriver* spid;

  thread_reference_t adis_Thd;
  float psc;
  float angle_vel;
  param_t offset; // float offset
  lpfilterStruct* lpf; //low pass filter for gyro

  volatile float angle; //Measured Gyro Angle In Rad

  uint8_t adis_gyroscope_not_calibrated;

} GyroStruct, *PGyroStruct;

extern int32_t gyro_comb;
extern int32_t gyro_temp;

PGyroStruct gyro_get(void);
PGyroStruct gyro_init(void);
//init ADIS gyro

int16_t gyro_get_raw_vel(PGyroStruct pGyro);
float gyro_get_offs(PGyroStruct pGyro);                         //read offset(result of callibration) for angular velocity
void gyro_update_offs(PGyroStruct pGyro, const int16_t offs);   //update offset from calibration value
void gyro_set_angle(const float angle);
uint16_t gyro_get_flash(PGyroStruct pGyro);                     //read number of flash for the rom un gyro
uint16_t gyro_get_power(PGyroStruct pGyro);                     //return milli-volt
uint16_t gyro_get_adc(PGyroStruct pGyro);                       //return milli-volt
uint16_t gyro_get_temp(PGyroStruct pGyro);                      //return milli-degree
gyro_state_t gyro_getState(void);                               //return state
uint8_t gyro_getError(void);                                    //return errorFlag

extern float yaw_pid_output_angle;

void polar_coordinate(int16_t x, int16_t y,int32_t current_angle);

extern float polar_angle ;
extern float polar_distance;

extern float Vx_by_polar;
extern float Vy_by_polar;

#endif /* INC_ADIS16265_H_ */
