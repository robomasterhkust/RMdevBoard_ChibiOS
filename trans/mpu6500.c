/**
 * This is device realize "read through write" paradigm. This is not
 * standard, but most of I2C devices use this paradigm.
 * You must write to device reading address, send restart to bus,
 * and then begin reading process.
 */

#include "ch.h"
#include "hal.h"

//#include "telemetry.h"
#include "mpu6500.h"
#include "flash.h"

#include "chprintf.h"
/* C libraries: */
#include <string.h>
#include <math.h>

#define GRAV                      9.81f

#define MPU6050_RX_BUF_SIZE       0x0E
#define MPU6050_TX_BUF_SIZE       0x05

/* MPU6050 useful registers */
#define MPU6050_SMPLRT_DIV        0x19
#define MPU6050_CONFIG            0x1A
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B
#define MPU6050_ACCEL_XOUT_L      0x3C
#define MPU6050_ACCEL_YOUT_H      0x3D
#define MPU6050_ACCEL_YOUT_L      0x3E
#define MPU6050_ACCEL_ZOUT_H      0x3F
#define MPU6050_ACCEL_ZOUT_L      0x40
#define MPU6050_TEMP_OUT_H        0x41
#define MPU6050_TEMP_OUT_L        0x42
#define MPU6050_GYRO_XOUT_H       0x43
#define MPU6050_GYRO_XOUT_L       0x44
#define MPU6050_GYRO_YOUT_H       0x45
#define MPU6050_GYRO_YOUT_L       0x46
#define MPU6050_GYRO_ZOUT_H       0x47
#define MPU6050_GYRO_ZOUT_L       0x48
#define MPU6050_PWR_MGMT_1        0x6B

/* I2C read transaction time-out in milliseconds. */
#define MPU6050_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6050_WRITE_TIMEOUT_MS  0x01

static const SPIConfig MPU6050_SPI_cfg =
{
  NULL,
  GPIO_CS,
  GPIO_Pin_CS,
  SPI_CR1_BR_0 | SPI_CR1_MSTR |
  SPI_CR1_CPHA | SPI_CR1_CPOL
};

/* IMU data structure. */
IMUStruct g_IMU1;

PIMUStruct mpu6050_get(void)
{
  return  &g_IMU1;
}

/**
 * Local variables
 */
/* Data buffers */
static int16_t mpu6050Data[7];

static uint8_t mpu6050RXData[MPU6050_RX_BUF_SIZE];
static uint8_t mpu6050TXData[MPU6050_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
static void imuStructureInit(PIMUStruct pIMU, IMUConfigStruct* imu_conf)
{
  uint8_t i;
  /* Initialize to zero. */
  memset((void *)pIMU, 0, sizeof(IMUStruct));

  switch(imu_conf->gyroConf)
  {
    case MPU6050_GYRO_SCALE_250:
      pIMU->gyro_psc = (1.0f / 131.0f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_500:
      pIMU->gyro_psc = (1.0f /  65.5f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_1000:
      pIMU->gyro_psc = (1.0f /  32.8f) * M_PI/180.0f;
      break;
    case MPU6050_GYRO_SCALE_2000:
      pIMU->gyro_psc = (1.0f /  16.4f) * M_PI/180.0f;
      break;
  }

  switch(imu_conf->accelConf)
  {
    case MPU6050_ACCEL_SCALE_2G:
      pIMU->accel_psc = (GRAV / 16384.0f);
      break;
    case MPU6050_ACCEL_SCALE_4G:
      pIMU->accel_psc = (GRAV /  8192.0f);
      break;
    case MPU6050_ACCEL_SCALE_8G:
      pIMU->accel_psc = (GRAV /  4096.0f);
      break;
    case MPU6050_ACCEL_SCALE_16G:
      pIMU->accel_psc = (GRAV /  2048.0f);
      break;
  }
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 0 - if reading was successful;
 *         1 - if reading failed.
 */
uint8_t mpu6050GetData(PIMUStruct pIMU)
{
  msg_t status = MSG_OK;

  /* Set the start register address for bulk data transfer. */
  mpu6050TXData[0] = MPU6050_ACCEL_XOUT_H;

  spiSelect(MPU6050_SPI);
  spiStartSend(MPU6050_SPI, 1, mpu6050TXData);
  spiStartReceive(MPU6050_SPI, 14, mpu6050RXData)
  spiUnselect(MPU6050_SPI);

  mpu6050Data[0] = (int16_t)((mpu6050RXData[ 0]<<8) | mpu6050RXData[ 1]); /* Accel X */
  mpu6050Data[1] = (int16_t)((mpu6050RXData[ 2]<<8) | mpu6050RXData[ 3]); /* Accel Y */
  mpu6050Data[2] = (int16_t)((mpu6050RXData[ 4]<<8) | mpu6050RXData[ 5]); /* Accel Z */
  mpu6050Data[3] = (int16_t)((mpu6050RXData[ 8]<<8) | mpu6050RXData[ 9]); /* Gyro X  */
  mpu6050Data[4] = (int16_t)((mpu6050RXData[10]<<8) | mpu6050RXData[11]); /* Gyro Y  */
  mpu6050Data[5] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Gyro Z  */
  mpu6050Data[6] = (int16_t)((mpu6050RXData[12]<<8) | mpu6050RXData[13]); /* Temperature  */

  /* X: */
  pIMU->accelData[X] = mpu6050Data[0] * pIMU->accel_psc;
  pIMU->gyroData[X]  = mpu6050Data[3] * pIMU->gyro_psc;

  /* Y: */
  pIMU->accelData[Y] = mpu6050Data[1] * pIMU->accel_psc;
  pIMU->gyroData[Y]  = mpu6050Data[4] * pIMU->gyro_psc;

  /* Z: */
  pIMU->accelData[Z] = mpu6050Data[2] * pIMU->accel_psc;
  pIMU->gyroData[Z]  = mpu6050Data[5] * pIMU->gyro_psc;

  return 0;
}

/**
 * @brief  Initialization function for the MPU6050 sensor.
 * @param  addr - I2C address of MPU6050 chip.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t mpu6050Init(PIMUStruct pIMU, IMUConfigStruct* imu_conf)
{
  msg_t status = MSG_OK;

  imuStructureInit(pIMU, imu_conf);

  spiAcquireBus(MPU6050_SPI);
  spiStart(MPU6050_SPI, &MPU6050_SPI_cfg);

  /* Reset all MPU6050 registers to their default values */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0xC0;          // Register value 0b11000000

  spiSelect(MPU6050_SPI);
  spiStartSend(MPU6050_SPI, 2, mpu6050TXData);
  spiUnselect(MPU6050_SPI);

  /* Wait 100 ms for the MPU6050 to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  mpu6050TXData[0] = MPU6050_PWR_MGMT_1;  // Start register address;
  mpu6050TXData[1] = 0x03;         // Register value CLKSEL = PLL_Z;

  spiSelect(MPU6050_SPI);
  spiStartSend(MPU6050_SPI, 2, mpu6050TXData);
  spiUnselect(MPU6050_SPI);

  /* Configure the MPU6050 sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  mpu6050TXData[0] = MPU6050_SMPLRT_DIV;  // Start register address;
  mpu6050TXData[1] = 11;                  // SMPLRT_DIV register value (8000 / (11 + 1) = 666 Hz);
  mpu6050TXData[2] = 0x00;          // CONFIG register value DLPF_CFG = 0 (256-260 Hz);
  mpu6050TXData[3] = imu_conf->gyroConf;          // GYRO_CONFIG register value FS_SEL = +-250 deg/s;
  mpu6050TXData[4] = imu_conf->accelConf;          // ACCEL_CONFIG register value AFS_SEL = +-4G

  spiSelect(MPU6050_SPI);
  spiStartSend(MPU6050_SPI, 5, mpu6050TXData);
  spiUnselect(MPU6050_SPI);

  return 0;
}
