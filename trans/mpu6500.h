#ifndef _MPU6050_H_
#define _MPU6050_H_

#define MPU6050_UPDATE_FREQ                    1000U  //Read MPU @ 50Hz
#define MPuU6050_FLASH_ADDR               0x08040000

#define MPU6500_SPI		                 	 &SPID5
#define GPIO_Pin_CS		       GPIOF_SPI5_IMU_NSS
#define GPIO_CS			                      GPIOF

#define MPU6050_GYRO_SCALE_250   0U
#define MPU6050_GYRO_SCALE_500   1U
#define MPU6050_GYRO_SCALE_1000  2U
#define MPU6050_GYRO_SCALE_2000  3U

#define MPU6050_ACCEL_SCALE_2G   0U
#define MPU6050_ACCEL_SCALE_4G   1U
#define MPU6050_ACCEL_SCALE_8G   2U
#define MPU6050_ACCEL_SCALE_16G  3U

#define MPU6050_UPDATE_PERIOD     1000000U/MPU6050_UPDATE_FREQ

typedef struct tagIMUStruct {
  float accelData[3];     /* Accelerometer data.             */
  float gyroData[3];      /* Gyroscope data.                 */
  float temperature;      /* IMU temperature.                 */

  float accelFiltered[3]; /* Filtered accelerometer data.    */
  float gyroFiltered[3];  /* Filtered gyro data.    */
  float accelBias[3];     /* Accelerometer bias.             */
  float gyroBias[3];      /* Gyroscope bias.                 */
  float transAccelBias[3]; /* when pitch >90degree            */
  float v2Filtered[3];    /* Filtered directionattr of gravity.  */
  float qIMU[4];          /* Attitude quaternion of the IMU. */

  uint16_t accel_psc;
  uint16_t gyro_psc;
} __attribute__((packed)) IMUStruct, *PIMUStruct;

typedef struct {
  const uint8_t accelConf;
  const uint8_t gyroConf;
} IMUConfigStruct;

#ifdef __cplusplus
extern "C" {
#endif
  PIMUStruct mpu6050_get(void);

  uint8_t mpu6050Init(PIMUStruct pIMU, IMUConfigStruct* imu_conf);
  uint8_t mpu6050GetData(PIMUStruct pIMU);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_H_ */
