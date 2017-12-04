#include "ch.h"
#include "hal.h"

#include "attitude.h"
#include "math_misc.h"

static float _error_int[3] = {0.0f, 0.0f, 0.0f};

uint8_t attitude_update(PIMUStruct pIMU)
{
  float corr[3] = {0.0f, 0.0f, 0.0f};
  float spinRate = vector_norm(pIMU->gyroData, 3);
  float accel = vector_norm(pIMU->accelData, 3);

  vector_normalize(pIMU->qIMU, 4);
  uint8_t i;

  if(accel < 12.81f && accel > 6.81f)
  {
    float accel_corr[3], norm_accel[3], v2[3];

    v2[0] = 2.0f * (pIMU->qIMU[1] * pIMU->qIMU[3] - pIMU->qIMU[0] * pIMU->qIMU[2]);
    v2[1] = 2.0f * (pIMU->qIMU[2] * pIMU->qIMU[3] + pIMU->qIMU[0] * pIMU->qIMU[1]);
    v2[2] = pIMU->qIMU[0] * pIMU->qIMU[0] -
            pIMU->qIMU[1] * pIMU->qIMU[1] -
            pIMU->qIMU[2] * pIMU->qIMU[2] +
            pIMU->qIMU[3] * pIMU->qIMU[3];

    for (i = 0; i < 3; i++)
      norm_accel[i] = pIMU->accelData[i]/accel;

    vector3_cross(norm_accel, v2, accel_corr);
    for (i = 0; i < 3; i++)
      corr[i] += accel_corr[i] * ATT_W_ACCEL;

    if(spinRate < 0.175f)
      for (i = 0; i < 3; i++)
      {
        _error_int[i] += corr[i] * (ATT_W_GYRO * pIMU->dt);

        if(_error_int[i] > GYRO_BIAS_MAX)
          _error_int[i] = GYRO_BIAS_MAX;
        if(_error_int[i] < -GYRO_BIAS_MAX)
          _error_int[i] = -GYRO_BIAS_MAX;
      }
  }

  for (i = 0; i < 3; i++)
    corr[i] += pIMU->gyroData[i] + _error_int[i];

  float dq[4];
  q_derivative(pIMU->qIMU, corr, dq);

  float q[4] = {pIMU->qIMU[0], pIMU->qIMU[1], pIMU->qIMU[2], pIMU->qIMU[3]};
  for (i = 0; i < 4; i++)
    q[i] += dq[i] * pIMU->dt;
  vector_normalize(q,4);

  if(isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3]))
  {
    for (i = 0; i < 4; i++)
      pIMU->qIMU[i] = q[i];

    #ifdef  IMU_USE_EULER_ANGLE
      quarternion2euler(pIMU->qIMU, pIMU->euler_angle);
    #endif

    return IMU_OK;
  }
  else
    return IMU_CORRUPTED_Q_DATA;
}

uint8_t attitude_imu_init(PIMUStruct pIMU)
{
  float rot_matrix[3][3];

  float norm = vector_norm(pIMU->accelData,3);
  uint8_t i;
  for (i = 0; i < 3; i++)
    rot_matrix[2][i] = pIMU->accelData[i] / norm;

  norm = sqrtf(rot_matrix[2][2]*rot_matrix[2][2] +
    rot_matrix[2][0]*rot_matrix[2][0]);

  rot_matrix[0][0] = rot_matrix[2][2] / norm;
  rot_matrix[0][1] = 0.0f;
  rot_matrix[0][2] = -rot_matrix[2][0] / norm;

  vector3_cross(rot_matrix[2], rot_matrix[0], rot_matrix[1]);
  rotm2quarternion(rot_matrix, pIMU->qIMU);

  return IMU_OK;
}
