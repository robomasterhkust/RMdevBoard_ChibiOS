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
#include "math_misc.h"
#include "chprintf.h"
/* C libraries: */
#include <string.h>

/* Mechanical mounting orientation parameters*/
typedef enum {
  IMU_X = 1U,
  IMU_Y = 0U,
  IMU_Z = 2U
} mpu_axis_mask_t;

#define MPU6500_RX_BUF_SIZE       0x0E
#define MPU6500_TX_BUF_SIZE       0x05

/* imu useful registers */
#define MPU6500_SMPLRT_DIV        0x19
#define MPU6500_CONFIG            0x1A
#define MPU6500_GYRO_CONFIG       0x1B
#define MPU6500_ACCEL_CONFIG_1    0x1C
#define MPU6500_ACCEL_CONFIG_2    0x20
#define MPU6500_ACCEL_XOUT_H      0x3B
#define MPU6500_ACCEL_XOUT_L      0x3C
#define MPU6500_ACCEL_YOUT_H      0x3D
#define MPU6500_ACCEL_YOUT_L      0x3E
#define MPU6500_ACCEL_ZOUT_H      0x3F
#define MPU6500_ACCEL_ZOUT_L      0x40
#define MPU6500_TEMP_OUT_H        0x41
#define MPU6500_TEMP_OUT_L        0x42
#define MPU6500_GYRO_XOUT_H       0x43
#define MPU6500_GYRO_XOUT_L       0x44
#define MPU6500_GYRO_YOUT_H       0x45
#define MPU6500_GYRO_YOUT_L       0x46
#define MPU6500_GYRO_ZOUT_H       0x47
#define MPU6500_GYRO_ZOUT_L       0x48
#define MPU6500_PWR_MGMT_1        0x6B

/* I2C read transaction time-out in milliseconds. */
#define MPU6500_READ_TIMEOUT_MS   0x01
/* I2C write transaction time-out in milliseconds. */
#define MPU6500_WRITE_TIMEOUT_MS  0x01

#define MPU6500_SENSOR_RESET      0x80
#define MPU6500_SENSOR_SLEEP      0x40
#define MPU6500_AUTO_SELECT_CLK   0x01

#define MPU6500_SPI_READ          0x80

#define TEMP_OFFSET               0

typedef enum{
  DLPF_250HZ  =  0,
  DLPF_184HZ  =  1,
  DLPF_92HZ   =  2,
  DLPF_41HZ   =  3,
  DLPF_20HZ   =  4,
  DLPF_10HZ   =  5,
  DLPF_5HZ    =  6,
  DLPF_3600HZ =  7
} mpu6500_dlpf_config_t;

typedef enum{
  ADLPF_460HZ =  0,
  ADLPF_184HZ =  1,
  ADLPF_92HZ  =  2,
  ADLPF_41HZ  =  3,
  ADLPF_20HZ  =  4,
  ADLPF_10HZ  =  5,
  ADLPF_5HZ   =  6
} mpu6500_acc_dlpf_config_t;

static const SPIConfig MPU6500_SPI_cfg =
{
  NULL,
  GPIOF,
  GPIOF_SPI5_IMU_NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR |
  SPI_CR1_CPHA | SPI_CR1_CPOL
};

/* IMU data structure. */
IMUStruct g_IMU1;

static lpfilterStruct lp_gyro_x;
static lpfilterStruct lp_gyro_y;
static lpfilterStruct lp_gyro_z;
#define GYRO_SAMPLE_FREQ   8000U
#define GYRO_CUTOFF_FREQ    500U

PIMUStruct imu_get(void)
{
  return  &g_IMU1;
}

/**
 * Local variables
 */
/* Data buffers */
static int16_t imuData[7];

static uint8_t imuRXData[MPU6500_RX_BUF_SIZE];
static uint8_t imuTXData[MPU6500_TX_BUF_SIZE];

/**
 * @brief  Initialization function of IMU data structure.
 * @param  pIMU - pointer to IMU data structure;
 * @param  fAddrLow - IMU address pin A0 is pulled low flag.
 */
static void imuStructureInit(PIMUStruct pIMU, IMUConfigStruct* imu_conf)
{
  memset((void *)pIMU, 0, sizeof(IMUStruct));
  pIMU->_imu_spi = imu_conf->_imu_spi;
  pIMU->imu_Thd = NULL;

  pIMU->_imu_spi = imu_conf->_imu_spi;

  params_set(pIMU->_accelBias, 31, 3, NULL, NULL, PARAM_PRIVATE);
  params_set(pIMU->_gyroBias, 30, 3, NULL, NULL, PARAM_PRIVATE);
  if(params_set(pIMU->_accelT[0], 29, 6, NULL, NULL, PARAM_PRIVATE) ||
     params_set(pIMU->_accelT[2], 28, 3, NULL, NULL, PARAM_PRIVATE))
  {
    pIMU->_accelT[0][0] = 1.0f;
    pIMU->_accelT[1][1] = 1.0f;
    pIMU->_accelT[2][2] = 1.0f;
    pIMU->_accelT[0][1] = 0.0f;
    pIMU->_accelT[1][2] = 0.0f;
    pIMU->_accelT[2][0] = 0.0f;
    pIMU->_accelT[0][2] = 0.0f;
    pIMU->_accelT[1][0] = 0.0f;
    pIMU->_accelT[2][1] = 0.0f;
  }

  switch(imu_conf->_gyroConf)
  {
    case MPU6500_GYRO_SCALE_250:
      pIMU->_gyro_psc = (1.0f / 131.0f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_500:
      pIMU->_gyro_psc = (1.0f /  65.5f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_1000:
      pIMU->_gyro_psc = (1.0f /  32.8f) * M_PI/180.0f;
      break;
    case MPU6500_GYRO_SCALE_2000:
      pIMU->_gyro_psc = (1.0f /  16.4f) * M_PI/180.0f;
      break;
  }

  switch(imu_conf->_accelConf)
  {
    case MPU6500_ACCEL_SCALE_2G:
      pIMU->_accel_psc = (GRAV / 16384.0f);
      break;
    case MPU6500_ACCEL_SCALE_4G:
      pIMU->_accel_psc = (GRAV /  8192.0f);
      break;
    case MPU6500_ACCEL_SCALE_8G:
      pIMU->_accel_psc = (GRAV /  4096.0f);
      break;
    case MPU6500_ACCEL_SCALE_16G:
      pIMU->_accel_psc = (GRAV /  2048.0f);
      break;
  }

  if(imu_conf->_axis_rev & MPU6500_AXIS_REV_X)
    pIMU->_axis_rev[X] = 1;
  if(imu_conf->_axis_rev & MPU6500_AXIS_REV_Y)
    pIMU->_axis_rev[Y] = 1;
  if(imu_conf->_axis_rev & MPU6500_AXIS_REV_Z)
    pIMU->_axis_rev[Z] = 1;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t imuGetData(PIMUStruct pIMU)
{
  uint32_t tcurr = chVTGetSystemTimeX();
  pIMU->dt = ST2US(tcurr - pIMU->_tprev)/1000000.0f;
  pIMU->_tprev = tcurr;

  float accelData[3],gyroData[3];
  uint8_t error =  imuGetDataRaw(pIMU, accelData, gyroData);

  if(!error)
  {
    float accelData_temp[3];

    accelData_temp[X] = accelData[X] - pIMU->_accelBias[X];
    accelData_temp[Y] = accelData[Y] - pIMU->_accelBias[Y];
    accelData_temp[Z] = accelData[Z] - pIMU->_accelBias[Z];

    //matrix33_multiply_vector3(pIMU->_accelT, accelData_temp, pIMU->accelData);
    matrix33_multiply_vector3(pIMU->_accelT, accelData_temp, accelData);

    if(pIMU->_axis_rev[X])
    {
      pIMU->gyroData[X] = -gyroData[X] - pIMU->_gyroBias[X];
      pIMU->accelData[X] = -accelData[X];
    }
    else
    {
      pIMU->gyroData[X] = gyroData[X] + pIMU->_gyroBias[X];
      pIMU->accelData[X] = accelData[X];
    }

    if(pIMU->_axis_rev[Y])
    {
      pIMU->gyroData[Y] = -gyroData[Y] - pIMU->_gyroBias[Y];
      pIMU->accelData[Y] = -accelData[Y];
    }
    else
    {
      pIMU->gyroData[Y] = gyroData[Y] + pIMU->_gyroBias[Y];
      pIMU->accelData[Y] = accelData[Y];
    }

    if(pIMU->_axis_rev[Z])
    {
      pIMU->gyroData[Z] = -gyroData[Z] - pIMU->_gyroBias[Z];
      pIMU->accelData[Z] = -accelData[Z];
    }
    else
    {
      pIMU->gyroData[Z] = gyroData[Z] + pIMU->_gyroBias[Z];
      pIMU->accelData[Z] = accelData[Z];
    }
  }

  return error;
}

/**
 * @brief  Reads new data from the sensor
 * @param  pIMU - pointer to IMU data structure;
 * @return 1 - if reading was successful;
 *         0 - if reading failed.
 */
uint8_t imuGetDataRaw(PIMUStruct pIMU, float AccelRaw[3], float GyroRaw[3])
{
  /* Set the start register address for bulk data transfer. */
  imuTXData[0] = MPU6500_ACCEL_XOUT_H | MPU6500_SPI_READ;
  spiAcquireBus(pIMU->_imu_spi);
  spiSelect(pIMU->_imu_spi);
  spiSend(pIMU->_imu_spi, 1, imuTXData);
  spiReceive(pIMU->_imu_spi, 14, imuRXData);
  spiUnselect(pIMU->_imu_spi);
	spiReleaseBus(pIMU->_imu_spi);

  imuData[0] = (int16_t)((imuRXData[ 0]<<8) | imuRXData[ 1]); /* Accel X */
  imuData[1] = (int16_t)((imuRXData[ 2]<<8) | imuRXData[ 3]); /* Accel Y */
  imuData[2] = (int16_t)((imuRXData[ 4]<<8) | imuRXData[ 5]); /* Accel Z */
  imuData[3] = (int16_t)((imuRXData[ 8]<<8) | imuRXData[ 9]); /* Gyro X  */
  imuData[4] = (int16_t)((imuRXData[10]<<8) | imuRXData[11]); /* Gyro Y  */
  imuData[5] = (int16_t)((imuRXData[12]<<8) | imuRXData[13]); /* Gyro Z  */
  imuData[6] = (int16_t)((imuRXData[ 6]<<8) | imuRXData[ 7]); /* Temperature */

  /* X: */
  AccelRaw[X] = (float)imuData[IMU_X] * pIMU->_accel_psc;
  GyroRaw[X]  = (float)imuData[IMU_X + 3] * pIMU->_gyro_psc;
  GyroRaw[X]  = lpfilter_apply(&lp_gyro_x, GyroRaw[X]);

  /* Y: */
  AccelRaw[Y] = (float)imuData[IMU_Y] * pIMU->_accel_psc;
  GyroRaw[Y]  = (float)imuData[IMU_Y + 3] * pIMU->_gyro_psc;
  GyroRaw[Y]  = lpfilter_apply(&lp_gyro_y, GyroRaw[Y]);

  /* Z: */
  AccelRaw[Z] = (float)imuData[IMU_Z] * pIMU->_accel_psc;
  GyroRaw[Z]  = (float)imuData[IMU_Z + 3] * pIMU->_gyro_psc;
  GyroRaw[Z]  = lpfilter_apply(&lp_gyro_z, GyroRaw[Z]);

  /* temperature */
  pIMU->temperature = (((float)imuData[6] - TEMP_OFFSET)/333.87) + 21;
  return IMU_OK;
}

/**
 * @brief  Initialization function for the imu sensor.
 * @return 1 - if initialization was successful;
 *         0 - if initialization failed.
 */
uint8_t imuInit(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf)
{
  imuStructureInit(pIMU, imu_conf);

  spiStart(pIMU->_imu_spi, &MPU6500_SPI_cfg);

  /* Reset all imu registers to their default values */
  imuTXData[0] = MPU6500_PWR_MGMT_1;  // Start register address;
  imuTXData[1] = MPU6500_SENSOR_RESET | MPU6500_SENSOR_SLEEP;

  spiAcquireBus(pIMU->_imu_spi);
  spiSelect(pIMU->_imu_spi);
  spiSend(pIMU->_imu_spi, 2, imuTXData);
  spiUnselect(pIMU->_imu_spi);
  spiReleaseBus(pIMU->_imu_spi);

  /* Wait 100 ms for the imu to reset */
  chThdSleepMilliseconds(100);

  /* Clear the SLEEP flag, set the clock and start measuring. */
  imuTXData[0] = MPU6500_PWR_MGMT_1;  // Start register address;
  imuTXData[1] = MPU6500_AUTO_SELECT_CLK;

  spiAcquireBus(pIMU->_imu_spi);
  spiSelect(pIMU->_imu_spi);
  spiSend(pIMU->_imu_spi, 2, imuTXData);
  spiUnselect(pIMU->_imu_spi);
  spiReleaseBus(pIMU->_imu_spi);


  /* Configure the imu sensor        */
  /* NOTE:                               */
  /* - SLEEP flag must be cleared before */
  /*   configuring the sensor.           */
  imuTXData[0] = MPU6500_CONFIG;  // Start register address;
  imuTXData[1] = DLPF_3600HZ;          // CONFIG register value DLPF_CFG;
  imuTXData[2] = (uint8_t)(imu_conf->_gyroConf << 3U);          // GYRO_CONFIG register value
  imuTXData[3] = (uint8_t)(imu_conf->_accelConf << 3U);          // ACCEL_CONFIG_1 register value
  imuTXData[4] = ADLPF_41HZ;          // ACCEL_CONFIG_2 register value

  spiAcquireBus(pIMU->_imu_spi);
  spiSelect(pIMU->_imu_spi);
  spiSend(pIMU->_imu_spi, 5, imuTXData);
  spiUnselect(pIMU->_imu_spi);
  spiReleaseBus(pIMU->_imu_spi);

  /* Add software 2rd order Butterworth filter for gyro */
  lpfilter_init(&lp_gyro_x, GYRO_SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
  lpfilter_init(&lp_gyro_y, GYRO_SAMPLE_FREQ, GYRO_CUTOFF_FREQ);
  lpfilter_init(&lp_gyro_z, GYRO_SAMPLE_FREQ, GYRO_CUTOFF_FREQ);

  pIMU->_tprev = chVTGetSystemTimeX();
  pIMU->inited = 1;
  return IMU_OK;
}
