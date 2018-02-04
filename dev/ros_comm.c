//
// Created by beck on 28/1/2018.
//

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "ros_comm.h"
#include "chstreams.h"

int ros_read()
{
    return chnGetTimeout((BaseChannel *)&SDU1, TIME_IMMEDIATE);
}

void ros_write(uint8_t* data, int length)
{
    chnWrite((BaseChannel *)&SDU1, data, length);
}

unsigned long ros_time()
{
    return ST2MS(chVTGetSystemTimeX());
}

int temp_buff_1;
uint8_t temp_buff[100];
static THD_WORKING_AREA(ros_comm_wa, 1024);
static THD_FUNCTION(ros_comm, p)
{
    while (!chThdShouldTerminateX())
    {
        temp_buff_1 = ros_read();
        chSequentialStreamRead((BaseChannel *)&SDU1, temp_buff, 100);
        chThdSleepMilliseconds(1);
    }
}

void ros_comm_init(void)
{
    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1000);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    chThdCreateStatic(  ros_comm_wa,
                        sizeof(ros_comm_wa),
                        NORMALPRIO,
                        ros_comm,
                        NULL);
}