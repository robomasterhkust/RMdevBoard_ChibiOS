/**
 * Edward ZHANG, 201710??
 * @file        calibrate_sensor.c
 * @brief       Sensor calibration functions
 * @reference   PX4
 */

#include "ch.h"
#include "hal.h"
#include "math_misc.h"

#include "mpu6500.h"
#include "adis16265.h"
#include "usbcfg.h"
#include "calibrate_sensor.h"
//#include "flash.h"
#include "chprintf.h"
#include <string.h>

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;

static uint8_t side_data_collected[SIDE_COUNT_MAX] = {false, false, false, false, false, false};
static uint8_t   side_complete_count          = 0;
static uint8_t   detect_attempt               = 0;
static uint8_t   calibration_state_record     = 0;

static float accel_ref[SIDE_COUNT_MAX][3];

/**
 * @brief detect the accelerometer direction
 * @require a_m_s2_sync after syncronized
 */
#define EMA_LEN            MS2ST(500) // exponential moving average time constant in seconds
#define NORMAL_STILL_THR        0.25f // normal still threshold
#define ACCEL_ERR_THR            5.0f // set accelerometer error threshold to 5m/s^2
#define STILL_THR2         powf(NORMAL_STILL_THR * 3, 2)

#define ACCEL_CAL_STILL_TIME_MS  500
#define ACCEL_CAL_TIMEOUT_S       10
static enum detect_orientation_return detect_orientation(PIMUStruct pIMU)
{
  float accel_ema[3] = {0.0f, 0.0f, 0.0f}; // exponential moving average of accel
  float accel_disp[3] = {0.0f, 0.0f, 0.0f}; // max-hold disperision of accel

  uint32_t still_time = MS2ST(ACCEL_CAL_STILL_TIME_MS);
  uint32_t t_start = chVTGetSystemTimeX();
  uint32_t timeout = S2ST(ACCEL_CAL_TIMEOUT_S);
  uint32_t t_timeout  = t_start + timeout;
  uint32_t t          = t_start;
  uint32_t t_prev     = t_start;
  uint32_t t_still    = t_start;

  uint8_t reach_still = false;

  uint16_t poll_err_count = 0;

  while (t <= (t_still + still_time) || !reach_still) {
    uint8_t i;
    // 1. calculate the weight for exponential moving average
    //  gettimeofday(&t_timeval, NULL);
    //  t = timerlong(&t_timeval);
    t = chVTGetSystemTimeX();

    uint16_t dt = t - t_prev;
    t_prev = t;
    float weight = (float)dt / (float)EMA_LEN;

    float accelData[3], gyroData[3];
    if (imuGetDataRaw(pIMU, accelData, gyroData) != IMU_OK) {
      chprintf(chp, "E:IMU Reading Error!\r\n");
      return DETECT_ORIENTATION_ERROR;
    }
    // 2. calculate the exponential moving average and max-hold disperision
    for (i = 0; i < 3; i++) {
      float di = accelData[i];

      float d  = di - accel_ema[i];
      accel_ema[i] += d * weight;
      d             = d * d;
      accel_disp[i] = accel_disp[i] * (1.0f - weight);
      if (d > STILL_THR2 * 8.0f)
        d = STILL_THR2 * 8.0f;

      if (d > accel_disp[i])
        accel_disp[i] = d;
    }

    // 3. still detector with hysteresis
    if ((accel_disp[X] < STILL_THR2) &&
        (accel_disp[Y] < STILL_THR2) &&
        (accel_disp[Z] < STILL_THR2)) {
      if (!reach_still) {
        chprintf(chp, "rest position reached, hold still\r\n");
        t_still     = t;
        t_timeout   = t + timeout;
        reach_still = true;
      }
    } else if ((accel_disp[X] > STILL_THR2 * 4.0f) ||
               (accel_disp[Y] > STILL_THR2 * 4.0f) ||
               (accel_disp[Z] > STILL_THR2 * 4.0f)) {
      if (reach_still) {
        chprintf(chp, "W:detected motion, hold still\r\n");
        reach_still = false;
        chThdSleepMilliseconds(100);
      }
    }

    if (t > t_timeout)
      poll_err_count++;
    if (poll_err_count > 1000) {
      chprintf(chp, "E:Detection for accelerometer failed, abort\r\n");
      return DETECT_ORIENTATION_ERROR;
    }
  }

  if ((fabsf(accel_ema[X] + GRAV) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z]) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Tail down\r\n");
    return DETECT_ORIENTATION_TAIL_DOWN;            // [ -g, 0, 0 ]
  }

  if ((fabsf(accel_ema[X] - GRAV) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z]) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Nose down\r\n");
    return DETECT_ORIENTATION_NOSE_DOWN;            // [ g, 0, 0 ]
  }


  if ((fabsf(accel_ema[X]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y] + GRAV) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z]) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Left\r\n");
    return DETECT_ORIENTATION_LEFT;            // [ 0, -g, 0 ]
  }


  if ((fabsf(accel_ema[X]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y] - GRAV) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z]) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Right\r\n");
    return DETECT_ORIENTATION_RIGHT;            // [ 0, g, 0 ]
  }


  if ((fabsf(accel_ema[X]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z] + GRAV) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Upside down\r\n");
    return DETECT_ORIENTATION_UPSIDE_DOWN;            // [ 0, 0, -g ]
  }

  if ((fabsf(accel_ema[X]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Y]) < ACCEL_ERR_THR) &&
      (fabsf(accel_ema[Z] - GRAV) < ACCEL_ERR_THR)) {
    chprintf(chp, "Detected orientation: Right side up\r\n");
    return DETECT_ORIENTATION_RIGHTSIDE_UP;            // [ 0, 0, g ]
  }

  return DETECT_ORIENTATION_ERROR;
}


/**
 * @brief reading the average reading from samples_num,
 *        fill the accel_ref for a single orientation
 * @require a_m_s2_sync updated on time
 * @TODO: include vibration detection in the future
 */
static uint8_t read_accelerometer_avg(PIMUStruct pIMU, const uint8_t orient, const uint32_t samples_num)
{
  uint32_t counts = 0;
  float accel_sum[3] = {0.0f, 0.0f, 0.0f};

  uint8_t i;

  float accelData[3], gyroData[3];

  const uint32_t sample_num_mod_20 = samples_num / 20;
  uint32_t count_samplenum_20 = sample_num_mod_20;
  chprintf(chp, "                      ]\r [");

  while (counts < samples_num) {
    if (imuGetDataRaw(pIMU, accelData, gyroData) != IMU_OK) {
      chprintf(chp, "E:IMU Reading Error!\r\n");
      return false;
    }
    for (i = 0; i < 3; i++)
      accel_sum[i] += accelData[i];
    chThdSleepMilliseconds(1);
    counts++;

    if (counts > count_samplenum_20) {
      chprintf(chp, "=");
      count_samplenum_20 += sample_num_mod_20;
    }
  }

  for (i = 0; i < 3; i++)
    accel_ref[orient][i] = accel_sum[i] / counts;

  chprintf(chp, "=\r\n");
  chprintf(chp, "On orient %d, the accel_ref for the imu: \r\n%f\r\n%f\r\n%f\r\n", orient,
           accel_ref[orient][X], accel_ref[orient][Y], accel_ref[orient][Z]);

  return true;
}

static uint8_t read_gyrscope_avg(PIMUStruct pIMU, const uint32_t samples_num)
{
  uint32_t counts = 0;
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

  uint8_t i;

  float accelData[3], gyroData[3];

  const uint32_t sample_num_mod_20 = samples_num / 20;
  uint32_t count_samplenum_20 = sample_num_mod_20;
  chprintf(chp, "                      ]\r [");

  while (counts < samples_num) {
    if (imuGetDataRaw(pIMU, accelData, gyroData) != IMU_OK) {
      chprintf(chp, "\nE:IMU Reading Error!\r\n");
      return false;
    }
    for (i = 0; i < 3; i++)
      gyro_sum[i] += gyroData[i];
    chThdSleepMilliseconds(1);
    counts++;

    if (counts > count_samplenum_20) {
      chprintf(chp, "=");
      count_samplenum_20 += sample_num_mod_20;
    }
  }

  chprintf(chp, "=\r\n");

  for (i = 0; i < 3; i++)
    pIMU->_gyroBias[i] = -gyro_sum[i] / counts;

  return true;
}


/*
 * @brief calculate offset and affine transformation for accelerometer
 * @require accel_ref vectors from read average
 * @output accel_T matrixs for affine rotation
 * @output accel_offs vectors for offset calculation
 */
static uint8_t calculate_calibration_values(PIMUStruct pIMU, float g)
{
  uint8_t i, j;
  for (i = 0; i < 3; i++)
    pIMU->_accelBias[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;

  /**
    * fill matrix A for linear equation A * x = b
    * A is the measurement without offset
    * x is the rotation matrix in Affine model
    * b is the reference matrix, [g 0 0; 0 g 0; 0 0 g] = eyes(3) * g
    */
  float mat_A[3][3];

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      mat_A[i][j] = accel_ref[i * 2 + 1][j] - pIMU->_accelBias[j];

  float mat_A_inv[3][3];

  if (!matrix_invert3(mat_A, mat_A_inv))
    return false;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      pIMU->_accelT[i][j] = mat_A_inv[i][j] * g;

  return true;
}

void calibrate_accelerometer(PIMUStruct pIMU)
{
  uint8_t i;

  enum detect_orientation_return orient = DETECT_ORIENTATION_ERROR;
  uint8_t read_avg_result;
  uint8_t calculation_result;

  calibration_state_record = STATE_DETECT_ORIENTATION;
  while (pIMU->accelerometer_not_calibrated) {
    // ESP_LOGI(TAG, "calibration task[%d], state[%d] event:", task_idx, calibration_state_record);
    switch (calibration_state_record) {
      case STATE_DETECT_ORIENTATION:
        orient = detect_orientation(pIMU);

            if ((orient == DETECT_ORIENTATION_ERROR)) {
              detect_attempt++;
              calibration_state_record = (detect_attempt < DETECT_ATTEMPT_MAX) ? STATE_DETECT_ORIENTATION
                                                                               : STATE_CALIBRATION_ERROR;
            } else {
              detect_attempt = 0;
              if (side_complete_count == SIDE_COUNT_MAX)
                calibration_state_record = STATE_CALCULATION;
              else if (side_data_collected[orient])
                calibration_state_record = STATE_DETECT_ORIENTATION;
              else
                calibration_state_record = STATE_READ_AVERAGE;
            }
            break;

      case STATE_READ_AVERAGE:
        chprintf(chp, "reading accelerometer on [%d]...\r\n", orient);
            read_avg_result = read_accelerometer_avg(pIMU, orient, 10000);
            if (!read_avg_result) {
              chprintf(chp, "W:IMU reading unsuccessful, redo reading average on [%d]\r\n", orient);
              calibration_state_record = STATE_READ_AVERAGE;
            } else {
              side_data_collected[orient] = true;
              side_complete_count++;
              chprintf(chp, "Orientation[%d] reading average done, %d side finished\r\n", orient, side_complete_count);
              calibration_state_record = STATE_DETECT_ORIENTATION;
            }
            break;

      case STATE_CALCULATION:
        calculation_result = calculate_calibration_values(pIMU, GRAV);
            if (!calculation_result) {
              chprintf(chp, "E:accelerometer has singularity, redo calibration\r\n");
              side_data_collected[0] = false;
              side_data_collected[1] = false;
              side_data_collected[2] = false;
              side_data_collected[3] = false;
              side_data_collected[4] = false;
              side_data_collected[5] = false;
              side_complete_count = 0;
              calibration_state_record = STATE_DETECT_ORIENTATION;
            }
            calibration_state_record = STATE_CALIBRATION_DONE;
            break;

      case STATE_CALIBRATION_DONE:
        pIMU->accelerometer_not_calibrated = false;
            chprintf(chp, "accelerometer calibration done.\r\n");

            break;

      case STATE_CALIBRATION_ERROR:
        pIMU->accelerometer_not_calibrated = false;
            chprintf(chp, "E:calibration error, closed the process.\r\n");
            break;

      default:
        break;
    }

    chThdSleepMilliseconds(100);
  }
}

void calibrate_gyroscope(PIMUStruct pIMU)
{
  calibration_state_record = STATE_READ_AVERAGE;
  while (pIMU->gyroscope_not_calibrated) {
    if ((calibration_state_record == STATE_READ_AVERAGE) && pIMU->gyroscope_not_calibrated) {
      chprintf(chp, "reading gyroscope, hold still...\r\n");
      pIMU->gyroscope_not_calibrated = !read_gyrscope_avg(pIMU, 10000);
      if (!pIMU->gyroscope_not_calibrated) {
        chprintf(chp, "gyroscope calibration done\r\n");
        chprintf(chp, "gyro offsets:\r\nX:%f\r\nY:%f\r\nZ:%f\r\n",
                 pIMU->_gyroBias[X], pIMU->_gyroBias[Y], pIMU->_gyroBias[Z]);

      }
    }

    chThdSleepMilliseconds(100);
  }
}


uint8_t calibrate_adi(PGyroStruct pGyro, const uint8_t full_cal)
{
  float gyro_zero = 0.0f;
  uint16_t i = 0;

  if (pGyro->state == NOT_INITED)
  {
    chprintf(chp,"Gyro not inited!\r\n");
    return -1;
  }

  int32_t sample_num;
  if(full_cal)
    sample_num = 50000;
  else
    sample_num = 1000;

  pGyro->state = CALIBRATING;
  chprintf(chp, "reading Gyro...\r\n ");
  chprintf(chp, "                    ]\r[");

  uint16_t count_samplenum_20 = 0;
  uint16_t sample_num_mod_20 = sample_num/20;

  for (i = 0; i < sample_num; i++)
  {
    gyro_zero += gyro_get_raw_vel(pGyro);

    if(i > count_samplenum_20)
    {
      chprintf(chp,"=");
      count_samplenum_20 += sample_num_mod_20;
    }

    chThdSleepMilliseconds(5);
  }

  gyro_zero *= pGyro->psc;
  gyro_zero /= -sample_num;
  gyro_zero -= pGyro->offset;

  if(!full_cal && fabsf(gyro_zero) > 0.1f)
  {
    chprintf(chp,"\rCalibration failed! please perform a full re-calibration\r\n");
    return -1;
  }

  chprintf(chp,"\r\n");
  chprintf(chp,"Calibration complete\r\n");
  pGyro->adis_gyroscope_not_calibrated = false;
  pGyro->offset += gyro_zero;
//  flashSectorErase(flashSectorAt(GYRO_CAL_FLASH));
//  flashWrite(GYRO_CAL_FLASH, &(pGyro->offset),4);

//  float test;
//  flashRead(GYRO_CAL_FLASH, &test,4);
  chprintf(chp,"gyro_offset: %f\r\n",  pGyro->offset ); //* 180.0f/M_PI

  pGyro->state = INITED;
  return 0;
}
