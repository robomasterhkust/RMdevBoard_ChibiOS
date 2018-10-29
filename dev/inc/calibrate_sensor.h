#ifndef __CALIBRATE_IMU_H_
#define __CALIBRATE_IMU_H_

#include "mpu6500.h"
#include "adis16265.h"
/**
 *  @brief calibration states record
 */
#define SIDE_COUNT_MAX                6

#define DETECT_ATTEMPT_MAX          100
enum detect_orientation_return // Declare the detection enum states
{
    DETECT_ORIENTATION_TAIL_DOWN,
    DETECT_ORIENTATION_NOSE_DOWN,
    DETECT_ORIENTATION_LEFT,
    DETECT_ORIENTATION_RIGHT,
    DETECT_ORIENTATION_UPSIDE_DOWN,
    DETECT_ORIENTATION_RIGHTSIDE_UP,
    DETECT_ORIENTATION_ERROR
};

enum calibration_state  // Declare the calibration enum states
{
    STATE_DETECT_ORIENTATION,
    STATE_READ_AVERAGE,
    STATE_CALCULATION,
    STATE_CALIBRATION_DONE,
    STATE_CALIBRATION_ERROR
};

void calibrate_gyroscope(PIMUStruct pIMU);
void calibrate_accelerometer(PIMUStruct pIMU);
uint8_t calibrate_adi(PGyroStruct pGyro, const uint8_t full_cal);

#endif
