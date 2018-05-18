/**
 * Edward ZHANG
 * @file    shellcfg.c
 * @brief   definitions of shell command functions
 */

#include "main.h"
#include "shellcfg.h"

#define SHELL_USE_USB

#if !defined(SHELL_USE_USB)
#define SHELL_SERIAL_UART &SD3

  static const SerialConfig shell_serial_conf = {
  115200,               //Baud Rate
  0,         //CR1 Register
  0,      //CR2 Register
  0                     //CR3 Register
};
#endif

static thread_t* matlab_thread_handler = NULL;
/**
 * @brief Transmit uint32_t and float through serial port to host machine
 * @require Initialization of ChibiOS serial driver before using this function
 *
 * @param[in] chp         pointer to a @p BaseSequentialStream implementing object
 * @param[in] txbuf_d     array of 32-bit integers to tramsmit, can be signed or unsigned
 * @param[in] txbuf_f     array of float point numbers to tramsmit
 * @param[in] num_int     number of 32-bit integers to tramsmit
 * @param[in] num_float   number of float point numbers to tramsmit
 *
 * @TODO improve the transmission protocol to enable easier setup for the host machine
 */
#define SYNC_SEQ  0xaabbccdd
static void transmit_matlab
  (BaseSequentialStream* chp,
    uint32_t* const txbuf_d, float* const txbuf_f,
    const uint8_t num_int, const uint8_t num_float)
{
  uint32_t sync = SYNC_SEQ;
  char* byte = (char*)&sync;

  uint8_t i;
  for (i = 0; i < 4; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_d;
  for (i = 0; i < 4*num_int; i++)
    chSequentialStreamPut(chp, *byte++);

  byte = (char*)txbuf_f;
  for (i = 0; i < 4*num_float; i++)
    chSequentialStreamPut(chp, *byte++);
}

#define HOST_TRANSMIT_FREQ  100U
static THD_WORKING_AREA(matlab_thread_wa, 512);
static THD_FUNCTION(matlab_thread, p)
{
  (void)p;
  chRegSetThreadName("matlab tramsmitter");

  int32_t txbuf_d[16];
  float txbuf_f[16];
  BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;

  PIMUStruct PIMU = imu_get();
  chassisStruct* chassis = chassis_get();
//  GimbalStruct* gimbal = gimbal_get();

  uint32_t tick = chVTGetSystemTimeX();
  const uint16_t period = US2ST(1000000/HOST_TRANSMIT_FREQ);
  while (!chThdShouldTerminateX())
  {
    tick += period;
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    txbuf_f[0] = chassis->_motors[FRONT_LEFT].speed_sp;
    txbuf_f[1] = chassis->_motors[FRONT_LEFT]._speed;

    transmit_matlab(chp, NULL, txbuf_f, 0, 2);
  }
}


extern volatile int32_t x_gyro;
extern volatile int32_t y_gyro;
extern volatile int32_t z_gyro;
extern volatile int32_t x_accl;
extern volatile int32_t y_accl;
extern volatile int32_t z_accl;
/*===========================================================================*/
/* Definitions of shell command functions                                    */
/*===========================================================================*/
static THD_WORKING_AREA(Shell_thread_wa, 1024);
void cmd_test(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
  PIMUStruct PIMU = imu_get();
//  GimbalStruct* gimbal = gimbal_get();
//  GimbalStruct* gimbal = get_gimbal_simple_controller();
    RC_Ctl_t* pRC = RC_get();
    volatile ChassisEncoder_canStruct* encoder = can_getChassisMotor();
//    imuStructADIS16470* imu_adis = imu_adis_get();

  chprintf(chp,"AccelX: %f\r\n",PIMU->accelData[X]);
  chprintf(chp,"AccelY: %f\r\n",PIMU->accelData[Y]);
  chprintf(chp,"AccelZ: %f\r\n",PIMU->accelData[Z]);

//  chprintf(chp,"Gimbal Pitch: %f\r\n",gimbal->motor[1]._angle);
//  chprintf(chp,"Gimbal Yaw: %f\r\n",gimbal->motor[0]._angle);
  chprintf(chp,"IMU Pitch: %f\r\n",PIMU->euler_angle[Pitch]);
    chprintf(chp, "rc command: %f\r\n", pRC->rc.channel0);

    chprintf(chp, "encoder speed: %f\r\n", encoder[2].raw_speed);

/*
 *  chprintf(chp, "adis16470 reading gyro x: %d \r\n", imu_adis->gyro_raw_data[0]);
    chprintf(chp, "adis16470 reading gyro y: %d \r\n", imu_adis->gyro_raw_data[1]);
    chprintf(chp, "adis16470 reading gyro z: %d \r\n", imu_adis->gyro_raw_data[2]);
    chprintf(chp, "adis16470 reading accl x: %d \r\n", imu_adis->accl_raw_data[0]);
    chprintf(chp, "adis16470 reading accl y: %d \r\n", imu_adis->accl_raw_data[1]);
    chprintf(chp, "adis16470 reading accl z: %d \r\n", imu_adis->accl_raw_data[2]);
    */
}

/**
 * @brief Start the data tramsmission to matlab
 * @note caution of data flooding to the serial port
 */
void cmd_data(BaseSequentialStream * chp, int argc, char *argv[])
{
  uint8_t sec = 10;

  if(argc && matlab_thread_handler == NULL)
  {
    char *toNumber = argv[0];
    uint32_t finalNum=0;
    while(*toNumber>='0' && *toNumber<='9')
      finalNum=finalNum*10+*(toNumber++)-'0';

    if(finalNum == 0)
      finalNum = 10;

    sec = (finalNum < 60 ? finalNum : 60);

    chprintf(chp,"Data transmission start in %d seconds...\r\n", sec);
    chThdSleepSeconds(sec);

    matlab_thread_handler = chThdCreateStatic(matlab_thread_wa, sizeof(matlab_thread_wa),
        NORMALPRIO - 3,
        matlab_thread, NULL);
  }
  else if(matlab_thread_handler != NULL)
  {
    chThdTerminate(matlab_thread_handler);
    matlab_thread_handler = NULL;
  }
}

void cmd_calibrate(BaseSequentialStream * chp, int argc, char *argv[])
{
    PIMUStruct pIMU = imu_get();
    PGyroStruct pGyro = gyro_get();

    if(argc)
    {
        if(!strcmp(argv[0], "accl"))
        {
            if(pIMU->state == IMU_STATE_READY)
            {
                pIMU->accelerometer_not_calibrated = true;

                pIMU->state == IMU_STATE_CALIBRATING;

                chThdSleepMilliseconds(10);
                calibrate_accelerometer(pIMU);
                chThdResume(&(pIMU->imu_Thd), MSG_OK);

                pIMU->state == IMU_STATE_READY;
            }
            else
                chprintf(chp, "IMU is not heated or the initialization not complete, connected to the battery first.\r\n");
        }
        else if(!strcmp(argv[0], "gyro"))
        {
            if(pIMU->state == IMU_STATE_READY)
            {
                pIMU->gyroscope_not_calibrated = true;

                pIMU->state == IMU_STATE_CALIBRATING;

                chThdSleepMilliseconds(10);
                calibrate_gyroscope(pIMU);
                chThdResume(&(pIMU->imu_Thd), MSG_OK);

                pIMU->state == IMU_STATE_READY;
            }
            else
                chprintf(chp, "IMU is not heated or the initialization not complete, connected to the battery first.\r\n");

        }
        else if(!strcmp(argv[0], "adi"))
        {
            pGyro->adis_gyroscope_not_calibrated = true;
            chThdSleepMilliseconds(10);
            calibrate_adi(pGyro,false); //fast calibration ~30s
            chThdResume(&(pGyro->adis_Thd), MSG_OK);
        }
        else if(!strcmp(argv[0], "adi-full"))
        {
            pGyro->adis_gyroscope_not_calibrated = true;
            chThdSleepMilliseconds(10);
            calibrate_adi(pGyro,true); //full calibration ~5min
            chThdResume(&(pGyro->adis_Thd), MSG_OK);
        }
        param_save_flash();
    }
    else
        chprintf(chp,"Calibration: gyro, accl, adi, adi-full\r\n");
}

void cmd_gimbal_encoder(BaseSequentialStream * chp, int argc, char *argv[])
{
    (void) argc,argv;
    GimbalStruct* gimbal = gimbal_get();
    chprintf(chp,"gimbalPitch: %f\r\n",gimbal->motor[GIMBAL_PITCH]._angle);
    chprintf(chp,"gimbalYaw:   %f\r\n",gimbal->motor[GIMBAL_YAW]._angle);

    chprintf(chp,"VelPitch: %f\r\n",gimbal->motor[GIMBAL_PITCH]._speed);
    chprintf(chp,"VelYaw:   %f\r\n",gimbal->motor[GIMBAL_YAW]._speed);
    chprintf(chp,"VelEncPitch: %f\r\n",gimbal->motor[GIMBAL_PITCH]._speed_enc);
    chprintf(chp,"VelEncYaw:   %f\r\n",gimbal->motor[GIMBAL_YAW]._speed_enc);
}

void cmd_temp(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
//  uint32_t tick = chVTGetSystemTimeX();
//  tick += US2ST(5U);

//  while(1){ // you can uncomment this so that it continuously send the data out.
              // this is useful in tuning the Temperature PID
      PIMUStruct _pimu = imu_get();
//      pTPIDStruct _tempPID = TPID_get();
      chprintf(chp,"%f\n", _pimu->temperature);
//      chprintf(chp,"Temperature: %f\f\n", _pimu->temperature);
//      chprintf(chp,"PID_value: %i\i\n", _tempPID->PID_Value);
//      chThdSleep(MS2ST(500));
//  }
}

void cmd_dbus(BaseSequentialStream * chp, int argc, char *argv[])
{
  (void) argc,argv;
//  uint32_t tick = chVTGetSystemTimeX();
//  tick += US2ST(5U);

  while(1){ // you can uncomment this so that it continuously send the data out.
              // this is useful in tuning the Temperature PID
//      PIMUStruct _pimu = imu_get();
      RC_Ctl_t* _pRC = RC_get();

//      pTPIDStruct _tempPID = TPID_get();
      chprintf(chp,"rc.channel0:%i\n",(int)_pRC->rc.channel0);

      chprintf(chp,"rc.channel1:%i\n",  (int)_pRC->rc.channel1);

      chprintf(chp,"rc.channel2:%i\n",(int)_pRC->rc.channel2);

      chprintf(chp,"rc.channel3:%i\n",  (int)_pRC->rc.channel3);

      chprintf(chp,"rc.s1: %i\n",(int)_pRC->rc.s1);

      chprintf(chp,"rc.s2: %i\n",(int)_pRC->rc.s2);

 //     chprintf(chp,"drive: %i\n", (int)*_pdrive);
//      chprintf(chp,"Temperature: %f\f\n", _pimu->temperature);
//      chprintf(chp,"PID_value: %i\i\n", _tempPID->PID_Value);
      chThdSleep(MS2ST(250));
  }
}

void cmd_gyro(BaseSequentialStream * chp, int argc, char *argv[])
{
      (void) argc,argv;

      PGyroStruct _pGyro = gyro_get();
      chprintf(chp,"Offset: %f\n", _pGyro->offset);
      chprintf(chp,"Angle_vel: %f\n", _pGyro->angle_vel);
      chprintf(chp,"Angle: %f\n", _pGyro->angle);
}

void cmd_ultrasonic(BaseSequentialStream * chp, int argc, char *argv[])
{
      (void) argc,argv;

//      float* pDist = hcsr04_getDistance();
//      chprintf(chp,"Distance: %f\n", *pDist);
}

/**
 * @brief array of shell commands, put the corresponding command and functions below
 * {"command", callback_function}
 */
static const ShellCommand commands[] =
{
  {"test", cmd_test},
  {"cal", cmd_calibrate},
  {"g", cmd_gimbal_encoder},
  {"\xEE", cmd_data},
//#ifdef MAVLINK_COMM_TEST
//  {"mavlink", cmd_mavlink},
//#endif
#ifdef PARAMS_USE_USB
  {"\xFD",cmd_param_scale},
  {"\xFB",cmd_param_update},
  {"\xFA",cmd_param_tx},
  {"\xF9",cmd_param_rx},
#endif
  {"temp", cmd_temp},
  {"dbus", cmd_dbus},
  {"gyro", cmd_gyro},
  {"ultra", cmd_ultrasonic},
//  {"error", cmd_error},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 =
{
  (BaseSequentialStream *)&SDU1,
  commands
};

/**
 * @brief start the shell service
 * @require enable the corresponding serial ports in mcuconf.h and board.h
 *          Select the SERIAL_CMD port in main.h
 *
 * @api
 */
void shellStart(void)
{
  //sdStart(SERIAL_CMD, NULL);
  /*
   * Initializes a serial-over-USB CDC driver.
   */
#ifdef SHELL_USE_USB
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */


  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);

  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
#else
    sdStart(SHELL_SERIAL_UART, shell_serial_conf);
#endif

  shellInit();

  shellCreateStatic(&shell_cfg1, Shell_thread_wa,
      sizeof(Shell_thread_wa), NORMALPRIO);

}
