//
//Created by Alex Wong on 2/6/2018
//

#ifndef INC_ADIS16470_H_
#define INC_ADIS16470_H_

#define ADIS16470_SPID          &SPID4

#define ADIS16470_pin           GPIOE_SPI4_ADIS_NSS
#define ADIS16470_port          GPIOE

#define ADIS16470_BURST_SIZE    10
#define ADIS16470_BUFFER_SIZE   ADIS16470_BURST_SIZE

#define ADIS16470_INTERNAL_CLK  0x0000
#define ADIS16470_EXTERNAL_CLK  0x0008

#define ADIS16470_UPDATE_PERIOD US2ST(500)

typedef struct data_16470_t{

    uint16_t diag_stat;     //For diagnostic
    int16_t x_gyro;         //1LSB = 0.1deg/s
    int16_t y_gyro;         //1LSB = 0.1deg/s
    int16_t z_gyro;         //1LSB = 0.1deg/s
    int16_t x_accl;         //1LSB = 1.25mg
    int16_t y_accl;         //1LSB = 1.25mg
    int16_t z_accl;         //1LSB = 1.25mg
    int16_t temperature;    //1LSB = 0.1deg C
    uint16_t data_cnt;      //1LSB = 500us

}__attribute__((packed)) data_16470_t;

typedef enum calRegAddr {

  GYROSCOPE = 0x40,
  ACCELEROMETER = 0x4C

}calRegAddr;

data_16470_t* adis16470data(void);
void adis16470_init(void);

#endif /* INC_ADIS16470_H_ */
