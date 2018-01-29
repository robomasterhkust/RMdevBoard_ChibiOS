/*
 * adis16265.c
 *
 *  Created on: 20 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"
#include "flash.h"
#include "math_misc.h"
#include "params.h"
#include "adis16265.h"

#if !defined(GYRO_ADIS)
#error "Unknown type of Gyro!"
#endif

static GyroStruct gyro;
static lpfilterStruct gyro_lpf;

int16_t gyro_cal_result = 0;

static const char gyro_cal_name[]  = "Gyro_cal";
static const char subname_gyro_cal[]  = "Gyro_Offset";

/* Private functions*/
void gyro_set_scale(PGyroStruct pGyro, const float scale);

PGyroStruct gyro_get(void)
{
  return &gyro;
}

#if defined(GYRO_INTERFACE_SPI)
static gyrodata_t gyro_txrx_spi(SPIDriver* const spid, const gyrodata_t data)
{
    gyrodata_t temp;

  spiAcquireBus(spid);
    spiSelect(spid);
  spiSend(spid, 1, &data);
  spiUnselect(spid);

  chThdSleepMicroseconds(10);

  spiSelect(spid);
  spiReceive(spid, 1, &temp);
  spiUnselect(spid);

  spiReleaseBus(spid);
  chThdSleepMicroseconds(10);
  return temp;
}
#endif

static void gyro_write(const PGyroStruct pGyro, const uint8_t addr, const gyrodata_t data)
{
  #if defined(GYRO_ADIS)
    gyrodata_t cmd1, cmd2;
      uint8_t address = ( ( addr & 0x3F ) | 0x80 );
      cmd1 = ( address << 8 ) | ( data >> 8 );
    cmd2 = ( (address-1) << 8 ) | ( data & 0x00FF);

    gyro_txrx_spi(pGyro->spid, cmd1 );
      gyro_txrx_spi(pGyro->spid, cmd2 );
  #endif
}

static uint16_t gyro_read(const PGyroStruct pGyro, const uint8_t addr )
{
  #if defined(GYRO_ADIS)
    uint16_t address = (0x3F & addr) << 8;
      address = gyro_txrx_spi(pGyro->spid, address );
  #endif

    return address;
}

static void gyro_update(PGyroStruct pGyro)
{
    float angle_raw = (float)(gyro_get_raw_vel(pGyro)) * pGyro->psc + pGyro->offset;
  float angle_vel;

  angle_vel = lpfilter_apply(pGyro->lpf, angle_raw);
  float angle = angle_vel * (GYRO_UPDATE_PERIOD_US / 1000000.0f);

  pGyro->angle += angle;
  pGyro->angle_vel = angle_vel;
}

void set_angle(PGyroStruct pGyro, const float angle )
{
    pGyro->angle = angle ;
}

float yaw_Kp = 1.5;
float yaw_Ki = 0;
float yaw_Kd = 0;

float yaw_temp_derivative = 0;
float yaw_temp_integral = 0;
float yaw_pre_error= 0;
float yaw_pid_output_angle = 0;

void yaw_axis_pid_cal(int32_t target_angle, int32_t current_angle){

    float error = target_angle - current_angle;
    float Kout = error * yaw_Kp;

    yaw_temp_integral += error;

    float Iout = yaw_temp_integral * yaw_Ki;

    yaw_temp_derivative = error - yaw_pre_error;

    float Dout = yaw_temp_derivative * yaw_Kd;

    yaw_pid_output_angle = Dout + Iout + Kout;

}

#if defined(GYRO_ADIS)
#define GYRO_OFFS_PSC    0.000319657f
void gyro_update_offs(PGyroStruct pGyro, const int16_t offs)
{
  if(pGyro->state != CALIBRATING)
    return;

  int16_t offset;
  uint16_t buf = 0;
  buf = gyro_read(pGyro, GYRO_OFF);

  if (buf & 0x0800 ) // 0b0000100000000000
    buf |= 0xF000; // 0b1111000000000000
  else
    buf &= 0x0FFF; // 0b0000111111111111;
  offset |= buf;
  offset += offs *(pGyro->psc/GYRO_OFFS_PSC);

  uint16_t* txbuf = (uint16_t*)&offset;
  gyro_write(pGyro, GYRO_OFF, *txbuf);
  gyro_write(pGyro, GYRO_COMD, 0x0008);
  chThdSleepMilliseconds(100); //nessasary. otherwise will fly
}

#define GYRO_SCALE_PSC    0.00048828f
void gyro_set_scale(PGyroStruct pGyro, const float scale)
{
  if(scale > 1.9995f || scale < 0.0f)
    return;

  int16_t scale_int = scale/GYRO_SCALE_PSC;
  uint16_t* txbuf = (uint16_t*)&(scale_int);
  gyro_write(pGyro, GYRO_SCALE, *txbuf);
}

float gyro_get_offs(PGyroStruct pGyro)
{
    uint16_t buf = 0;
    int16_t off = 0;
    buf = gyro_read(pGyro, GYRO_OFF);

    if (buf & 0x0800 ) // 0b0000100000000000
        buf |= 0xF000; // 0b1111000000000000
    else
        buf &= 0x0FFF; // 0b0000111111111111;
    off |=buf;

    return (float)(off * GYRO_OFFS_PSC);
}

int16_t gyro_get_raw_vel(PGyroStruct pGyro)
{
    uint16_t buf = 0;
      int16_t vel = 0;
    buf = gyro_read(pGyro, GYRO_VEL);
    if (buf & 0x2000 ) // 0b0010000000000000)
        buf |= 0xC000;  //0b1100000000000000
    else
        buf &= 0x3FFF; //0b0011111111111111;
    vel |=buf;

    return vel;
}

uint16_t gyro_get_angle(PGyroStruct pGyro)
{
    uint16_t angle = 0;
    angle = gyro_read(pGyro, GYRO_ANGL) & 0x3FFF;   //0b0011111111111111;
    return angle;
}

uint16_t gyro_get_flash(PGyroStruct pGyro)
{
    gyro_read(pGyro, GYRO_FLASH);
    return gyro_read(pGyro, GYRO_FLASH);
}

uint16_t gyro_get_power(PGyroStruct pGyro)
{
    gyro_read(pGyro, GYRO_POWER);
    return 1832*(gyro_read(pGyro, GYRO_POWER) & 0x0FFF);
}

uint16_t gyro_get_adc(PGyroStruct pGyro)
{
    gyro_read(pGyro,GYRO_ADC);
    return 6105*(gyro_read(pGyro, GYRO_ADC) & 0x0FFF);
}

uint16_t gyro_get_temp(PGyroStruct pGyro)
{
    gyro_read(pGyro, GYRO_TEMP);
    return 145*(gyro_read(pGyro, GYRO_TEMP) & 0x0FFF);
}
#endif

#define GYRO_UPDATE_PERIOD  US2ST(GYRO_UPDATE_PERIOD_US)
static THD_WORKING_AREA(gyro_thread_wa, 1024);
static THD_FUNCTION(gyro_thread,p)
{
  PGyroStruct pGyro = (PGyroStruct)p;
  chRegSetThreadName("External Yaw Gyro");

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += GYRO_UPDATE_PERIOD;
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pGyro->error_flag |= GYRO_LOSE_FRAME;
    }

    if(pGyro->state == INITED)
      gyro_update(pGyro);

    if(pGyro->adis_gyroscope_not_calibrated)
        {
          chSysLock();
          chThdSuspendS(&(pGyro->adis_Thd));
          chSysUnlock();
        }
  }
}

PGyroStruct gyro_init(void)
{
  PGyroStruct pGyro = &gyro;
  memset((void *)pGyro, 0, sizeof(GyroStruct));

  pGyro->lpf = &gyro_lpf;
  pGyro->angle = 0.0f;
  pGyro->angle_vel = 0.0f;
  pGyro->adis_Thd = NULL;
  params_set(&pGyro->offset, 27, 1, gyro_cal_name, subname_gyro_cal, PARAM_PRIVATE);
 // flashRead(GYRO_CAL_FLASH, &(pGyro->offset), 27);
  //if(!isfinite(pGyro->offset))
   // pGyro->offset = 0.0f;

  //Set low pass filter at cutoff frequency 44Hz
  lpfilter_init(pGyro->lpf, (float)GYRO_UPDATE_FREQ, 12.0f);

  #if defined(GYRO_ADIS)
    pGyro->spid = GYRO_SPI;
    const SPIConfig GyroSPI_cfg =
    {
      NULL,
      GYRO_NSS_port,
      GYRO_NSS_pin,
      SPI_CR1_MSTR | //SPI_Mode_Master
      SPI_CR1_DFF |  //SPI_DataSize_16b
      SPI_CR1_BR_2 | //SPI_BaudRatePrescaler_32
      SPI_CR1_CPHA | //SPI_CPHA_2Edge
      SPI_CR1_CPOL   //SPI_CPOL_High
    };

    pGyro->spid->rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    pGyro->spid->txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spiStart(pGyro->spid, &GyroSPI_cfg);

    //Reset.....
    gyro_write(pGyro, GYRO_COMD,0x0080);        // sofware reset
      chThdSleepMilliseconds(50);

    //Factory Cal...
    gyro_write(pGyro, GYRO_COMD, 0x0002);

    //Set Filter...
    uint16_t gyro_conf = 0;
    gyro_conf |= 1U << (10 - GYRO_PSC_CONF);

    gyro_write(pGyro, GYRO_SENS, gyro_conf);
    pGyro->psc = GYRO_PSC / (float)(1U << GYRO_PSC_CONF);

    gyro_write(pGyro, GYRO_SMPL,0x0000);        // Internal Sample Freq 1/1.953 ~= 500Hz
    gyro_write(pGyro, GYRO_COMD,0x0008);        // Auxiliary DAC data latch
      chThdSleepMilliseconds(100);
  #endif

  chThdCreateStatic(gyro_thread_wa,sizeof(gyro_thread_wa),
          NORMALPRIO + 5,
          gyro_thread, &gyro);

    pGyro->state = INITED;

  return pGyro;
}

/*
 * State:
 *  NOT_INITED = 0,
 *  INITED = 1,
 *  CALIBRATING = 2
 */

gyro_state_t gyro_getState(void){
  return gyro.state;
}
/*
 * Error List:
 * 0: Not Inited
 * 1: No Error
 */
uint8_t gyro_getError(void){
  return gyro.error_flag;
}
