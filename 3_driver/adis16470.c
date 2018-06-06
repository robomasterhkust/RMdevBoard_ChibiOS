//
//Created by Alex Wong on 2/6/2018
//Tested on 2018 Type-A RM Dev Board
//
//Connection:
//MISO PE5
//MOSI PE6
//NSS  PE4
//SEK  PE12
//

#include "ch.h"
#include "hal.h"
#include "adis16470.h"

uint16_t rxbuf[ADIS16470_BUFFER_SIZE];

static data_16470_t burst_data;

static const SPIConfig adis16470SpiCfg = {

  NULL,
  ADIS16470_port,
  ADIS16470_pin,
  SPI_CR1_MSTR | SPI_CR1_DFF | SPI_CR1_BR_2 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL

};

static void burstRead(void) {

  static const uint16_t burst_cmd = 0x6800;

  spiAcquireBus(ADIS16470_SPID);            //request burst read
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &burst_cmd);

  spiReceive(ADIS16470_SPID, ADIS16470_BUFFER_SIZE, &rxbuf);    //receive data
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

  uint8_t i = 0;                            //calculate checksum
  uint8_t checksum = 0;
  uint8_t* buf_addr = (uint8_t*)&rxbuf;
  for (i = 0; i < (ADIS16470_BURST_SIZE - 1) * 2; i++) {
    checksum += *buf_addr;
    buf_addr++;
  }

  if (checksum == (uint8_t)rxbuf[9]) {
    memcpy(&burst_data, &rxbuf, (ADIS16470_BURST_SIZE - 1) * 2);
  }

}

static uint16_t readword(uint8_t address) {

  uint16_t data = 0;
  data = ((uint16_t)address) << 8;

  spiAcquireBus(ADIS16470_SPID);            //request from user specified address
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &data);
  spiUnselect(ADIS16470_SPID);

  chThdSleep(US2ST(16));                    //stall time specified by data sheet

  spiSelect(ADIS16470_SPID);                //read word
  spiReceive(ADIS16470_SPID, 1, &data);
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

  return data;

}

static void writeword(uint8_t lowerAddress, uint16_t data) {

  uint16_t txbuffer[2];

  //concatenate write bit, address and data to be written
  txbuffer[0] = 0x8000 | lowerAddress << 8 | (uint8_t)data;     //pack lower word
  txbuffer[1] = 0x8000 | (lowerAddress + 1) << 8 | data >> 8;   //pack higher word

  spiAcquireBus(ADIS16470_SPID);
  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &txbuffer);        //send lower word
  spiUnselect(ADIS16470_SPID);

  chThdSleep(US2ST(16));                        //stall time specified by data sheet

  spiSelect(ADIS16470_SPID);
  spiSend(ADIS16470_SPID, 1, &txbuffer[1]);     //send higher word
  spiUnselect(ADIS16470_SPID);
  spiReleaseBus(ADIS16470_SPID);

}

static void setBias(calRegAddr sensorType, int32_t xbias, int32_t ybias, int32_t zbias) {

  writeword(sensorType, (uint16_t) xbias);
  writeword(sensorType + 2, (uint16_t) xbias >> 16);
  writeword(sensorType + 4, (uint16_t) ybias);
  writeword(sensorType + 6, (uint16_t) ybias >> 16);
  writeword(sensorType + 8, (uint16_t) zbias);
  writeword(sensorType + 10, (uint16_t) zbias >> 16);

}

//#define CALIBRATE_ADIS16470

static THD_WORKING_AREA(adis16470Thd_wa, 1024);
static THD_FUNCTION(adis16470Thd, p) {

  (void)p;

  //set clock source
  uint16_t mscCtrl = readword(0x60);                          //read MSC_CTRL register
  mscCtrl = (mscCtrl & (~0x001C)) | ADIS16470_INTERNAL_CLK;   //set bits [2:4], mask the rest
  writeword(0x60, mscCtrl);                                   //set MSC_CTRL with new command

  //set filter to be the same in beck's code
  writeword(0x5C, 0x0004);

  //calibrate imu by setting bias, comment out if not in use
  //ozone graphing windows provides average readout, useful for calibration
#ifdef CALIBRATE_ADIS16470
  setBias(GYROSCOPE, 0, 0, 0);
  setBias(ACCELEROMETER, 0, 0, 0);
  writeword(0x68, 0x0008);          //save to flash
#endif

  while(!chThdShouldTerminateX()) {

    burstRead();

    chThdSleep(ADIS16470_UPDATE_PERIOD);

  }

}

data_16470_t* adis16470data(void){

  return &burst_data;

}

void adis16470_init(void) {

  memset((void*)&rxbuf, 0, ADIS16470_BUFFER_SIZE);
  memset((void*)&burst_data, 0, sizeof(data_16470_t));

  (*ADIS16470_SPID).rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  (*ADIS16470_SPID).txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
  spiStart(ADIS16470_SPID, &adis16470SpiCfg);

  chThdCreateStatic(adis16470Thd_wa, sizeof(adis16470Thd_wa),
                    NORMALPRIO + 7, adis16470Thd, NULL);

}

