//
// Created by beck on 2/4/2018.
//

#include "ch.h"
#include "hal.h"
#include "attitude_estimator_task.h"
#include "imu_temp.h"

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);

/**
 * Initialize the accelerometer, gyroscope, and magnetometer
 *  And start the attitude calculation and sensor fusion.
 *  Also start the temperature control thread
 * @related: driver/mpu6500.c, driver/adis16265.c, driver/ist8310.c
 * @related: dev/imu_temp.h
 */
// TODO: sensor selection and error message
static THD_FUNCTION(Attitude_thread, p)
{
    chRegSetThreadName("IMU Attitude Estimator");

    (void) p;

    PIMUStruct pIMU = imu_get();    // mpu6500 tri-axial imu
    PGyroStruct pGyro = gyro_get(); // adis16265 single axial gyroscope

    // Initialize accelerometer and gyroscope from the main SPI bus
    static const IMUConfigStruct imu1_conf =
            {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_250, MPU6500_AXIS_REV_X};
    imuInit(pIMU, &imu1_conf);
    chThdSleepMilliseconds(50);

    // Initialize ADIS16265 single axial gyroscope
    // TODO: check if ADIS16265 exist
//    gyro_init();

//    // Initialize magnetometer from the additional I2C bus
//    static const magConfigStruct mag1_conf =
//            {IST8310_ADDR_FLOATING, 200, IST8310_AXIS_REV_NO};
//    ist8310_init(&mag1_conf);

    // Check temperature feedback before starting temp controller thread
    imuGetData(pIMU);
    if (pIMU->temperature > 0.0f)
        tempControllerInit();
    else
        pIMU->errorCode |= IMU_TEMP_ERROR;

    while(pIMU->temperature < IMU_TEMP_SETPOINT)
    {
        imuGetData(pIMU);
        chThdSleepMilliseconds(100);
    }

    pIMU->state = IMU_STATE_READY;
    attitude_imu_init(pIMU);

    uint32_t tick = chVTGetSystemTimeX();

    while (!chThdShouldTerminateX()) {
        // Hardware coded synchronization
        tick += US2ST(MPU6500_UPDATE_PERIOD_US);
        if (chVTGetSystemTimeX() < tick)
            chThdSleepUntil(tick);
        else {
            tick = chVTGetSystemTimeX();
            pIMU->errorCode |= IMU_LOSE_FRAME;
        }

        if(pIMU->temperature < IMU_TEMP_SETPOINT - 6.0f || pIMU->temperature > IMU_TEMP_SETPOINT + 6.0f )
            pIMU->errorCode |= IMU_TEMP_WARNING;

        imuGetData(pIMU);
//        ist8310_update();

        // TODO: sensor selection
        // if adis 16470 exist

        // else if adis16265 exist
//        attitude_update_fused(pIMU, pGyro);
        // else
        attitude_estimator_mpu6500_init(pIMU);

        if (pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated) {
            chSysLock();
            chThdSuspendS(&(pIMU->imu_Thd));
            chSysUnlock();
        }
    }
}


void attitude_estimator_init(void)
{
    chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa),
                      NORMALPRIO + 5, Attitude_thread, NULL);
}