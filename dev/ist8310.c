/**
 * Edward ZHANG, 20171217
 * @file    ist8310.c
 * @brief   ist8310 magnetometer driver
 */

#include "ch.h"
#include "hal.h"

#include "ist8310.h"

#ifdef IMU_SLAVE
  #include "mpu6500.h"
#endif

#define IST8310_STAT1      0x02
#define IST8310_XOUT_L     0x03
#define IST8310_XOUT_H     0x04
#define IST8310_YOUT_L     0x05
#define IST8310_YOUT_H     0x06
#define IST8310_ZOUT_L     0x07
#define IST8310_ZOUT_H     0x08
#define IST8310_STAT2      0x09
#define IST8310_CTRL1      0x0A
#define IST8310_CTRL2      0x0B
#define IST8310_TEMP_OUT_L 0x1C
#define IST8310_TEMP_OUT_H 0x1D

#define IST8310_STAT1_DOR  0x02
#define IST8310_STAT1_DRDY 0x01

#define IST8310_CTRL2_DREN 0x08
#define IST8310_CTRL2_DRP  0x04
#define IST8310_CTRL2_SRST 0x01

/* ist8310 magnetometer data structure*/
static magStruct ist8310;

magStruct* ist8310_get(void)
{
  return &ist8310;
}

volatile float* ist8310_getValue(void)
{
  return ist8310.data;
}

uint32_t ist8310_getError(void)
{
  return ist8310.errorFlag;
}

static void magStructInit(const magConfigStruct* const conf)
{
  ist8310._offset[X] = 0.0f;
  ist8310._offset[Y] = 0.0f;
  ist8310._offset[Z] = 0.0f;

  if(conf->axis_rev & IST8310_AXIS_REV_X)
    ist8310._axis_rev[X] = 1;
  if(conf->axis_rev & IST8310_AXIS_REV_Y)
    ist8310._axis_rev[Y] = 1;
  if(conf->axis_rev & IST8310_AXIS_REV_Z)
    ist8310._axis_rev[Z] = 1;
}

/*
 * @brief Initialize IST8310
 * @NOTE  If the sensor is connected as the slave of IMU, we need to initialize IMU first
 */
uint8_t ist8310_init(const magConfigStruct* const conf)
{
  #ifdef IMU_SLAVE
    PIMUStruct pIMU = imu_get();
    ist8310._spi = pIMU->_imu_spi;

    uint8_t data[5];
    data[0] = MPU6500_I2C_MST_CTRL;
    data[1] = MPU6500_I2CMST_CLK_400K; //set imu master i2c freq to 320kHz;
    data[2] = conf->addr;                    //I2C_SLV0_ADDR
    data[3] = IST8310_CTRL2;           //I2C_SLV0_REG
    data[4] = MPU6500_I2C_MSTR_EN | 6; //I2C_SLV0_CTRL

    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 5, data);
    spiUnselect(ist8310._spi);
    spiReleaseBus(ist8310._spi);

    data[0] = MPU6500_USER_CFG;
    data[1] = MPU6500_USER_I2C_MST;
    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 2, data);
    spiUnselect(ist8310._spi);
    spiReleaseBus(ist8310._spi);

    IST8310_RESET();
    chThdSleepMilliseconds(100);
    IST8310_SET();

    data[0] = MPU6500_I2C_SLV0_REG;
    data[1] = IST8310_CTRL1;
    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 2, data);
    spiUnselect(ist8310._spi);
    spiReleaseBus(ist8310._spi);

    data[0] = MPU6500_I2C_SLV0_DO;
    switch(conf->sample_rate)
    {
      case IST8310_SINGLE_MEASUREMENT: data[1] = 1;  break;
      case 8:                          data[1] = 2;  break;
      case 10:                         data[1] = 3;  break;
      case 20:                         data[1] = 5;  break;
      case 100:                        data[1] = 6;  break;
      case 50:                         data[1] = 7;  break;
      case IST8310_SAMPLE_RATE_1_2HZ:  data[1] = 9;  break;
      case 1:                          data[1] = 10; break;
      case 200:                        data[1] = 11; break;
      default:
        ist8310.errorFlag |= IST8310_INVALID_SAMPLE_RATE;
        return 1;
    }
    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 2, data);
    spiUnselect(ist8310._spi);
    spiReleaseBus(ist8310._spi);

    chThdSleepMilliseconds(2);

    data[0] = MPU6500_I2C_SLV0_ADDR;
    data[1] = conf->addr | MPU6500_I2C_MSTR_READ;
    data[2] = IST8310_XOUT_L;
    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 3, data);
    spiUnselect(ist8310._spi);
    spiReleaseBus(ist8310._spi);

    ist8310._inited = true;
    return 0;
  #else
  #endif
}

static inline uint8_t ist8310_getRawValue(uint16_t* rawData)
{
  #ifdef IMU_SLAVE
    uint8_t data = MPU6500_EXT_SENS_DATA | MPU6500_I2C_MSTR_READ;

    spiAcquireBus(ist8310._spi);
    spiSelect(ist8310._spi);
    spiSend(ist8310._spi, 1, &data);
    spiReceive(ist8310._spi, 6, (uint8_t*)rawData);
    spiUnselect(ist8310._spi);
  	spiReleaseBus(ist8310._spi);

    return 0;
  #else
  #endif
}

static void trans_magBias(void)
{
  uint8_t i;
  for (i = 0; i < 3; i++)
  {
    ist8310.data[i] -= ist8310._offset[i];
  }
}

/*
 *  @brief This function updates the reading from the ist8310 magnetometer
 */
uint8_t ist8310_update(void)
{
  uint16_t rawData[3];
  uint8_t result = ist8310_getRawValue(rawData);

  uint8_t i;
  for (i = 0; i < 3; i++)
  {
    if(ist8310._axis_rev[i])
      ist8310.data[i] = (float)(-rawData[i]) * IST8310_PSC;
    else
      ist8310.data[i] = rawData[i] * IST8310_PSC;
  }
  trans_magBias();

  return result;
}
