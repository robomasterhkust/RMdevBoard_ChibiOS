/*MiceTrack.c
 *
 *
 *  Collect the adometry data from teensy3.6 by I2C protocal
 *
 */

#include "ch.h"
#include "hal.h"
#include "MiceTrack.h"
#include "i2c.h"

#define _RX_BUF_SIZE       0x0E
#define _TX_BUF_SIZE       0x0E

#define _Address     0x19

static uint8_t BoardTXData[_TX_BUF_SIZE];
static uint8_t BoardRXData[_RX_BUF_SIZE];
static uint32_t getDistance = 0;
static int Distance;   //received msg

static  THD_WORKING_AREA(MiceTrackThread_wa, 128);

static const I2CConfig i2cfg = {
                                OPMODE_I2C,
                                400000,
                                FAST_DUTY_CYCLE_2
};

void StartTransmitData(I2CDriver *i2cp, const I2CConfig *i2cfg){

    i2cAcquireBus(i2cp);

    i2cMasterTransmitTimeout(i2cp, _Address ,BoardTXData , 4,
                              BoardRXData, 4, TIME_INFINITE);

    //convert an array of uint8 data to uint32
    getDistance = (((uint32_t)BoardRXData[3]) << 24) |
        (((uint32_t)BoardRXData[2]) << 16)
         | (((uint32_t)BoardRXData[1]) << 8)
         | ((uint32_t)BoardRXData[0]);
    Distance = (int)(getDistance - 32767)

    BoardTXData[0]= BoardRXData[0]; //call back
    BoardTXData[1]= BoardRXData[1];
    BoardTXData[2]= BoardRXData[2];
    BoardTXData[3]= BoardRXData[3];

    i2cReleaseBus(i2cp);
  }


static THD_WORKING_AREA(MICETRACK_THREAD, 128);

static THD_FUNCTION(TransmitDataTrd, arg){

  (void) arg;

  while(1){
    StartTransmitData(&I2CD2,&i2cfg);
    chThdSleepMilliseconds(50);
  }
}

void startMiceTrack(void)
{
  i2cObjectInit(&I2CD2);
  i2cStart(&I2CD2, &i2cfg);

  chThdCreateStatic(MICETRACK_THREAD, sizeof(MiceTrackThread_wa),
                    NORMALPRIO, TransmitDataTrd, NULL);

}




