//
// Created by beck on 2/4/2018.
//

#ifndef RM_CHIBIOS_ADIS16470_H
#define RM_CHIBIOS_ADIS16470_H

#define ADIS16470_RESET_PIN         GPIOI_PIN9
#define ADIS16470_RESET_PORT        GPIOI
#define ADIS16470_RESET_TIME        100
#define ADIS16470_SPI               SPID4

/****  ADIS16470 internal registers   ****/
#define ADIS_X_GYRO_LOW         0x04

typedef struct imuStructADIS16470 {
    float gyro_raw_data[3];
    float accl_raw_data[3];
    float temperature;
    float time_stamp;

    float deltang[3];
    float deltvel[3];
    float g_bias[3];
    float a_bias[3];

}imuStructADIS16470, *imuStructADIS16470Ptr;


#endif //RM_CHIBIOS_ADIS16470_H
