#ifndef _MPU6500_H_
#define _MPU6500_H_

#define MPU6500_UPDATE_FREQ                    1000U  //Read MPU @ 1000Hz
#define IMU_USE_EULER_ANGLE
//#define IMU_ACCL_USE_LPF

#include "params.h"

typedef enum {
  X = 0U,
  Y = 1U,
  Z = 2U
} axis_mask_t;

typedef enum {
  Roll = 0U,
  Pitch = 1U,
  Yaw = 2U
} mpu_euler_angle_t;

typedef enum {
  MPU6500_I2C_ADDR_A0_LOW = 0x68,
  MPU6500_I2C_ADDR_A0_HIGH = 0x69
} mpu_i2c_addr_t;

typedef enum {
  MPU6500_GYRO_SCALE_250 = 0,
  MPU6500_GYRO_SCALE_500 = 1,
  MPU6500_GYRO_SCALE_1000 = 2,
  MPU6500_GYRO_SCALE_2000 = 3
} mpu_gyro_scale_t;

typedef enum {
  MPU6500_ACCEL_SCALE_2G = 0,
  MPU6500_ACCEL_SCALE_4G = 1,
  MPU6500_ACCEL_SCALE_8G = 2,
  MPU6500_ACCEL_SCALE_16G = 3
} mpu_accel_scale_t;

typedef enum {
  MPU6500_AXIS_REV_NO = 0,
  MPU6500_AXIS_REV_X = 1,
  MPU6500_AXIS_REV_Y = 2,
  MPU6500_AXIS_REV_Z = 4,
} mpu_axis_rev_t;

typedef enum {
  IMU_OK = 0,
  IMU_CORRUPTED_Q_DATA = 1<<1,
  IMU_TEMP_ERROR = 1<<2,
  IMU_TEMP_WARNING = 1<<30,
  IMU_LOSE_FRAME = 1<<31
} imu_att_error_t;

typedef enum {
  MPU6500_I2CMST_CLK_348K = 0,
  MPU6500_I2CMST_CLK_333K = 1,
  MPU6500_I2CMST_CLK_320K = 2,
  MPU6500_I2CMST_CLK_308K = 3,
  MPU6500_I2CMST_CLK_296K = 4,
  MPU6500_I2CMST_CLK_286K = 5,
  MPU6500_I2CMST_CLK_276K = 6,
  MPU6500_I2CMST_CLK_267K = 7,
  MPU6500_I2CMST_CLK_258K = 8,
  MPU6500_I2CMST_CLK_500K = 9,
  MPU6500_I2CMST_CLK_471K = 10,
  MPU6500_I2CMST_CLK_444K = 11,
  MPU6500_I2CMST_CLK_421K = 12,
  MPU6500_I2CMST_CLK_400K = 13,
  MPU6500_I2CMST_CLK_381K = 14,
  MPU6500_I2CMST_CLK_364K = 15
} mpu_i2cmst_clk_t;

#define MPU6500_I2C_MSTR_EN          0x80
#define MPU6500_I2C_MSTR_BYTE_SWAP   0x40
#define MPU6500_I2C_MSTR_REG_DIS     0x20
#define MPU6500_I2C_MSTR_GRP         0x10

#define MPU6500_SPI_READ             0x80
#define MPU6500_I2C_MSTR_READ        0x80

#define MPU6500_EXT_SENS_DATA        0x49
#define MPU6500_USER_CFG             0x6A
#define MPU6500_I2C_MST_CTRL         0x24
#define MPU6500_I2C_SLV0_ADDR        0x25
#define MPU6500_I2C_SLV0_REG         0x26
#define MPU6500_I2C_SLV0_CTRL        0x27
#define MPU6500_I2C_SLV0_DO          0x63
#define MPU6500_I2C_SLV1_ADDR        0x28
#define MPU6500_I2C_SLV1_REG         0x29
#define MPU6500_I2C_SLV1_CTRL        0x2A
#define MPU6500_I2C_SLV1_DO          0x64
#define MPU6500_I2C_SLV2_ADDR        0x2B
#define MPU6500_I2C_SLV2_REG         0x2C
#define MPU6500_I2C_SLV2_CTRL        0x2D
#define MPU6500_I2C_SLV2_DO          0x65
#define MPU6500_I2C_SLV3_ADDR        0x2E
#define MPU6500_I2C_SLV3_REG         0x2F
#define MPU6500_I2C_SLV3_CTRL        0x30
#define MPU6500_I2C_SLV3_DO          0x66

#define MPU6500_USER_I2C_MST         0x20

#define IMU_ERROR_COUNT    1U
#define IMU_WARNING_COUNT  1U
static const char imu_error_messages[][IMU_ERROR_COUNT] =
{
  "E:Corrupted IMU Q data",
};

static const char imu_warning_messages[][IMU_WARNING_COUNT] =
{
  "W:IMU Reading lose frame"
};

typedef enum{
  IMU_STATE_UNINIT = 0,
  IMU_STATE_HEATING,
  IMU_STATE_CALIBRATING,
  IMU_STATE_READY
} imu_state_t;

#define MPU6500_UPDATE_PERIOD     1000000U/MPU6500_UPDATE_FREQ

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float temperature;      /* IMU temperature.                 */

  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float gyroFiltered[3];  /* Filtered gyro data.    */

  float qIMU[4];          /* Attitude quaternion of the IMU. */

  #ifdef  IMU_USE_EULER_ANGLE
    float euler_angle[3];      /* Euler angle of the IMU. */
    float d_euler_angle[3];    /* Euler angle changing rate of the IMU. */

    int rev;
    float prev_yaw;         /* used to detect zero-crossing */
  #else
    float dqIMU[4];         /* Attitude quaternion changing rate of the IMU. */
  #endif

  param_t _accelBias[3];    /* Accelerometer bias.             */
  param_t _accelT[3][3];    /* Accelerometer rotational bias matrix       */
  param_t _gyroBias[3];     /* Gyroscope bias.                 */

  bool  _axis_rev[3];
  float _accel_psc;
  float _gyro_psc;

  SPIDriver* _imu_spi;
  imu_state_t state;
  uint32_t errorCode;
  uint32_t _tprev;
  float dt;
  thread_reference_t imu_Thd;

  uint8_t accelerometer_not_calibrated;
  uint8_t gyroscope_not_calibrated;
} __attribute__((packed)) IMUStruct, *PIMUStruct;

typedef struct {
  SPIDriver* const _imu_spi;
  const mpu_accel_scale_t _accelConf;
  const mpu_gyro_scale_t _gyroConf;
  const uint8_t _axis_rev;
} IMUConfigStruct;

#ifdef __cplusplus
extern "C" {
#endif

PIMUStruct imu_get(void);

uint8_t imuInit(PIMUStruct pIMU, const IMUConfigStruct* const imu_conf);
uint8_t imuGetDataRaw(PIMUStruct pIMU, float AccelRaw[3], float GyroRaw[3]);
uint8_t imuGetData(PIMUStruct pIMU);

#ifdef __cplusplus
#endif

#endif /* _MPU6500_H_ */
