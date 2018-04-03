//
// Created by beck on 2/4/2018.
//

#include "ch.h"
#include "hal.h"
#include "string.h"
#include "adis16470.h"

/**
 * pseudo code
 *
 * main(void)
 *
 * spi start with spi_config
 *
 * spi write to registers to set up
 *
 * spi read some registers to initial trial
 *
 * while(1)
 * {
 *      spi read some registers directly
 * }
 */

#define GYRO_SPI                &SPID4
static const SPIConfig adis16470_spi_cfg =
        {
                NULL,
                GPIOE,
                GPIOE_SPI4_ADIS_NSS,
                SPI_CR1_MSTR | //SPI_Mode_Master
                SPI_CR1_DFF  | //SPI_DataSize_16b
                SPI_CR1_BR_2 | SPI_CR1_BR_0 | //SPI_BaudRatePrescaler_32
                SPI_CR1_CPHA | //SPI_CPHA_2Edge
                SPI_CR1_CPOL   //SPI_CPOL_High
        };

static imuStructADIS16470 imu;

void imu_init_adis16470(void)
{
    imuStructADIS16470Ptr imuPtr = &imu;
    memset((void*)imuPtr, 0, sizeof(imuStructADIS16470));

    /*
     * Initialize the SPI bus
     * 1. Hardware reset;
     * 2. configure the SPI bus;
     * 3. set up the DMA
     */
    palClearPad(ADIS16470_RESET_PORT, ADIS16470_RESET_PIN);
    chThdSleepMilliseconds(ADIS16470_RESET_TIME / 2);
    palSetPad(ADIS16470_RESET_PORT, ADIS16470_RESET_PIN);
    chThdSleepMilliseconds(ADIS16470_RESET_TIME / 2);

    ADIS16470_SPI.rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    ADIS16470_SPI.txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spiStart(&ADIS16470_SPI, &adis16470_spi_cfg);

    /*
     * IMU inner related registers
     */

}