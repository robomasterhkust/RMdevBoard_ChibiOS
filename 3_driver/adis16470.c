//
// Created by beck on 2/4/2018.
//

#include "ch.h"
#include "hal.h"
#include "string.h"
#include "adis16470.h"

uint8_t rxbuf_receive[ADIS16470_IMU_BURST_LENGTH];
imuStructADIS16470 imu_adis;
volatile uint16_t gyro_low;

static const SPIConfig adis16470_spi_cfg =
        {
                NULL,
                GPIOE,
                GPIOE_SPI4_ADIS_NSS,
                SPI_CR1_MSTR |  //SPI_Mode_Master
                SPI_CR1_DFF |   //SPI_DataSize_16Bit
                //168MHz, APB2 84MHz, BR[110]: fPCLK/128
                SPI_CR1_BR_2 | SPI_CR1_BR_1 | // 656.25kHz
                SPI_CR1_CPHA |  //SPI_CPHA_2Edge, rising edge is the MSBit
                SPI_CR1_CPOL    //SPI_CPOL_High, high-level idle state
                // MSB-first by not set LSBFIRST
                // Not binary communication by not set SPI_CR1_BIDIMODE
        };


/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
static uint16_t spi_read_register(SPIDriver *spip, uint8_t reg)
{
    spiSelect(spip);
    uint16_t txbuf = reg << (uint16_t)8 | (uint16_t)0x00;
    uint16_t rxbuf;
    spiExchange(spip, 1, &txbuf, &rxbuf);
    spiUnselect(spip);
    return rxbuf;
}

/**
 * @brief Write the register in an revised order for 16 bits each
 * Write to IMU inner related registers
 *      1. NR / W = 1
 *      2. Address [ A6: A0]
 *      3. New data[DC7:DC0]
 * @param regAddr
 * @param regData
 * @return
 */
static int spi_write_register(SPIDriver *spip, uint8_t regAddr, int16_t regData)
{
    uint16_t addr = (((regAddr & (uint16_t)0x7F) | (uint16_t)0x80) << 8);
    uint16_t lowerWord =  (addr                     | ( regData         & (uint16_t)0xFF)); // OR register address (A) with data(D), AADD
    uint16_t upperWord = ((addr | (uint16_t)0x100)  | ((regData >> 8)   & (uint16_t)0xFF)); // OR register address with data and increment the address

    /*
     * Write to IMU inner related registers
     * 1. NR / W = 1
     * 2. Address [ A6: A0]
     * 3. New data[DC7:DC0]
     */
    spiSelect(spip);
    spiSend(spip, 1, &lowerWord);
    spiUnselect(spip);
    spiSelect(spip);
    spiSend(spip, 1, &upperWord);
    spiUnselect(spip);
    return 0;
}

/**
 * @brief   Burst Reads for ADIS16470
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip          pointer to the SPI initerface
 * @param[in] reg           hardcoded burst register 0x6800
 * @param[out]rxbuf_output  The received buffer value.
 */
static uint8_t spi_burst_read(SPIDriver *spip, uint8_t *rxbuf_output)
{
    spiSelect(spip);
    uint16_t txbuf_16bit[2];
    txbuf_16bit[0] = 0x6800;
    spiSend(spip, 1, txbuf_16bit);
    txbuf_16bit[0] = 0x0000; // NR/W bit to 1
//    spiReceive(spip, ADIS16470_IMU_BURST_LENGTH, rxbuf_output);
    for (int i = 0; i < ADIS16470_IMU_BURST_LENGTH; ++i) {
        spiExchange(spip, 1, &txbuf_16bit[0], rxbuf_output+i);
    }
    spiUnselect(spip);
    return rxbuf_output[1];
}

/**
 * @brief Decode and verify the received message from SPI
 * @param[in] rxbuf     message buffer from the burst message
 * @param[out] imuPtr
 */
static void decode_burst_message(const uint8_t * rxbuf, imuStructADIS16470Ptr imuPtr)
{
    imuPtr->checksum        = rxbuf[18];
    uint8_t cur_checksum    = 0;
    for (int i = 0; i < ADIS16470_IMU_BURST_LENGTH - 1; ++i) {
        cur_checksum += rxbuf[i];
    }
    if (cur_checksum == imuPtr->checksum) {
        imuPtr->diag_stat       = (uint16_t)rxbuf[1] | (uint16_t)rxbuf[0] >> 8;
        imuPtr->gyro_raw_data[0]= (uint16_t)rxbuf[3] | (uint16_t)rxbuf[2] >> 8;
        imuPtr->gyro_raw_data[1]= (uint16_t)rxbuf[5] | (uint16_t)rxbuf[4] >> 8;
        imuPtr->gyro_raw_data[2]= (uint16_t)rxbuf[7] | (uint16_t)rxbuf[6] >> 8;
        imuPtr->accl_raw_data[0]= (uint16_t)rxbuf[9] | (uint16_t)rxbuf[8] >> 8;
        imuPtr->accl_raw_data[1]= (uint16_t)rxbuf[11]| (uint16_t)rxbuf[10] >> 8;
        imuPtr->accl_raw_data[2]= (uint16_t)rxbuf[13]| (uint16_t)rxbuf[12] >> 8;
        imuPtr->temperature     = (uint16_t)rxbuf[15]| (uint16_t)rxbuf[14] >> 8;
        imuPtr->data_count      = (uint16_t)rxbuf[17]| (uint16_t)rxbuf[16] >> 8;
        // TODO: decode diag_stat
    }
    else {
        // Throw error on checksum
        imuPtr->diag_stat       = 0xFF;
    }
}

static THD_WORKING_AREA(imu_adis16470_driver_wa, 1024);

static THD_FUNCTION(imu_adis16470_driver, p)
{
    imuStructADIS16470Ptr imuPtr = (imuStructADIS16470Ptr)p;
    chRegSetThreadName("External ADIS16470 IMU driver");

    uint32_t tick = chVTGetSystemTimeX();
    while (!chThdShouldTerminateX()) {
        tick += ADIS16470_IMU_UPDATE_PERIOD;
        if (chVTGetSystemTimeX() < tick) {
            chThdSleepUntil(tick);
        } else {
            tick = chVTGetSystemTimeX();
        }
        /*
         * Burst mode, continous read
         * spi_write_16bit(ADIS_IMU_BURST_DIN);
         * spi_read
         */
//        spi_burst_read(EXTERNAL_SPI_Ptr, rxbuf_receive);
//        decode_burst_message(rxbuf_receive, imuPtr);
//        rxbuf_receive[0] = spi_read_single_register(EXTERNAL_SPI_Ptr, 0x04);
        gyro_low = spi_read_register(EXTERNAL_SPI_Ptr, ADIS16470_IMU_X_GYRO_LOW);
    }

}

void imu_init_adis16470(void)
{
    imuStructADIS16470Ptr imuPtr = &imu_adis;
    memset((void *) imuPtr, 0, sizeof(imuStructADIS16470));

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

    *EXTERNAL_SPI_Ptr.rxdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    *EXTERNAL_SPI_Ptr.txdmamode |= STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD;
    spiStart(EXTERNAL_SPI_Ptr, &adis16470_spi_cfg);


    /*
     * Filter Control Register (FILT_CTRL)
     * number of taps in each stage of the Bartlett window FIR filter
     */
    spi_write_register(EXTERNAL_SPI_Ptr, ADIS16470_IMU_FILT_CTRL,
                        0b00000100);

    /*
     * Miscellaneous Control Register (MSC_CTRL)
     * 7. linear g compensation for gyroscope enabled,
     * 6. Point of percussion alignment enabled,
     * 4:2. internal clock mode
     * 1. SYNC polarity falling edge triggers sampling
     * 0. DR polarity active high when data is valid
     */
    spi_write_register(EXTERNAL_SPI_Ptr, ADIS16470_IMU_MSC_CTRL,
                       0b11000001);

    /*
     * Decimation Filter (DEC_RATE), disabled
     */
    spi_write_register(EXTERNAL_SPI_Ptr, ADIS16470_IMU_DEC_RATE,
                       0x00);

    chThdCreateStatic(imu_adis16470_driver_wa, sizeof(imu_adis16470_driver_wa),
                      NORMALPRIO,
                      imu_adis16470_driver, imuPtr);

}