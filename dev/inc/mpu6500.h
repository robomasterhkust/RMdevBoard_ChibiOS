#ifndef _MPU6500_H_
#define _MPU6500_H_

#define MPU6500_UPDATE_FREQ                    1000U  //Read MPU @ 1000Hz
#define IMU_CAL_FLASH                     0x08040000

typedef enum {
  X = 0U,
  Y = 1U,
  Z = 2U
} mpu_axis_mask_t;

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
  IMU_OK = 0,
  IMU_I2C_ERROR1 = 1,
  IMU_I2C_ERROR2 = 2,
  IMU_I2C_ERROR3 = 3,
  IMU_ATT_TIMEOUT = 4,
  IMU_CORRUPTED_Q_DATA = 5
} imu_att_error_t;

#define MPU6500_UPDATE_PERIOD     1000000U/MPU6500_UPDATE_FREQ

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float temperature;      /* IMU temperature.                 */

  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float gyroFiltered[3];  /* Filtered gyro data.    */

  float qIMU[4];          /* Attitude quaternion of the IMU. */
  float euler_angle[3];   /* Euler angle of the IMU. */
  float d_euler_angle[3]; /* Euler angle changing rate of the IMU. */
  float _attitude_err_int[3];

  float _accelBias[3];     /* Accelerometer bias.             */
  float _accelT[3][3];     /* Accelerometer rotational bias matrix       */
  float _gyroBias[3];      /* Gyroscope bias.                 */

  float _accel_psc;
  float _gyro_psc;

  SPIDriver* _imu_spi;
  uint8_t inited;
  uint8_t data_invalid;
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
