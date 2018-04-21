//
// Created by beck on 2/4/2018.
//

#ifndef RM_CHIBIOS_ADIS16470_H
#define RM_CHIBIOS_ADIS16470_H

#define ADIS16470_RESET_PIN         GPIOI_PIN9
#define ADIS16470_RESET_PORT        GPIOI
#define ADIS16470_RESET_TIME        100
#define EXTERNAL_SPI_Ptr            &SPID4

/****  driver parameter  ****/
#define ADIS16470_IMU_UPDATE_FREQ       1000U
#define ADIS16470_IMU_UPDATE_PERIOD_US  1000000U/ADIS16470_IMU_UPDATE_FREQ
#define ADIS16470_IMU_UPDATE_PERIOD     US2ST(ADIS16470_IMU_UPDATE_PERIOD_US)
#define ADIS16470_IMU_BURST_LENGTH      19

/****  ADIS16470 command  ****/
#define ADIS16470_IMU_BURST_DIN_UPPER   0x68

/****  ADIS16470 user register memory map   ****/
#define ADIS16470_IMU_DIAG_STAT     0x02
#define ADIS16470_IMU_X_GYRO_LOW    0x04
#define ADIS16470_IMU_X_GYRO_OUT    0x06
#define ADIS16470_IMU_Y_GYRO_LOW    0x08
#define ADIS16470_IMU_Y_GYRO_OUT    0x0A
#define ADIS16470_IMU_Z_GYRO_LOW    0x0C
#define ADIS16470_IMU_Z_GYRO_OUT    0x0E
#define ADIS16470_IMU_X_ACCL_LOW    0x10
#define ADIS16470_IMU_X_ACCL_OUT    0x12
#define ADIS16470_IMU_Y_ACCL_LOW    0x14
#define ADIS16470_IMU_Y_ACCL_OUT    0x16
#define ADIS16470_IMU_Z_ACCL_LOW    0x18
#define ADIS16470_IMU_Z_ACCL_OUT    0x1A
#define ADIS16470_IMU_TEMP_OUT      0x1C
#define ADIS16470_IMU_TIME_STAMP    0x1E

/****  ADIS16470 initial register  ****/
#define ADIS16470_IMU_FILT_CTRL    	0x5C  //Filter control
#define ADIS16470_IMU_MSC_CTRL    	0x60  //Miscellaneous control
#define ADIS16470_IMU_UP_SCALE    	0x62  //Clock scale factor, PPS mode
#define ADIS16470_IMU_DEC_RATE    	0x64  //Decimation rate control (output data rate)
#define ADIS16470_IMU_NULL_CFG    	0x66  //Auto-null configuration control
#define ADIS16470_IMU_GLOB_CMD    	0x68  //Global commands

#define ADIS16470_GYRO_RESOLU_32BIT 0.00000152587 // 0.1 / 2^16 in degree

typedef struct imuStructADIS16470 {
    uint16_t diag_stat;
    int32_t gyro_raw_data[3];
    int32_t accl_raw_data[3];
    int16_t temperature;
    uint16_t data_count;
    uint8_t checksum;
    bool verified;

    double gyro[3];
    double accl[3];
    float deltang[3];
    float deltvel[3];
    float g_bias[3];
    float a_bias[3];

}imuStructADIS16470, *imuStructADIS16470Ptr;

typedef enum {
    RESERVED = 0,
    DATA_PATH_OVERRUN = 1,
    FLASH_MEMORY_UPDATE_FAILURE = 2,
    SPI_COMM_ERROR = 3,
    STANDBY_MODE_VOLTAGE_LOW = 4,
    SENSOR_FAILURE = 5,
    MEMORY_FAILURE = 6,
    CLOCK_ERROR = 7
} ADIS_IMU_DIAG_STAT_BITS;

#ifdef __cplusplus
extern "C" {
#endif

imuStructADIS16470Ptr imu_adis_get(void);
//void imu_adis_read(void);
void imu_init_adis16470(void);

#ifdef __cplusplus
}
#endif

#endif //RM_CHIBIOS_ADIS16470_H
