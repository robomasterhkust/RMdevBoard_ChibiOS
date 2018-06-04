/**
 * Beck Pang, 20180405
 * @brief       onboard CAN driver configuration file and receiver threads
 * @reference   canBusProcess.c
 */
#include "ch.h"
#include "hal.h"
#include "can_bus.h"

/*
 * 500KBaud, Automatic Bus-Off Management,
 * automatic wakeup, automatic recover from abort mode.
 * @reference   section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
        // HAL LIB, hcan1.Init.ABOM = DISABLE; hcan1.Init.AWUM = DISABLE;
        CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
        CAN_BTR_SJW(0) | CAN_BTR_TS2(3) |
        CAN_BTR_TS1(8) | CAN_BTR_BRP(2)

};

static CANFilter canfilter[CAN_FILTER_NUM];

/**
 * private function to distribute the message to different tasks
 */
static void can_process_encoder_message(CANDriver *const canp, const CANRxFrame * rxmsg)
{
    if (canp == &CAND1) {
        switch (rxmsg->SID) {
            case CAN_C620_STD_ID_FEEDBACK_MSG_ID_1:
            case CAN_C620_STD_ID_FEEDBACK_MSG_ID_2:
            case CAN_C620_STD_ID_FEEDBACK_MSG_ID_3:
            case CAN_C620_STD_ID_FEEDBACK_MSG_ID_4:
                can_process_chassis_encoder(rxmsg);
                break;
            case CAN_GIMBAL_YAW_FEEDBACK_MSG_ID:
            case CAN_GIMBAL_PITCH_FEEDBACK_MSG_ID:
                can_process_gimbal_encoder(rxmsg);
                break;
            case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_7:
            case CAN_C620_EXTRA_ID_FEEDBACK_MSG_ID_8:
                can_process_extra_encoder(rxmsg);
                break;
            case CAN_UWB_MSG_ID:
                // can_process_uwb(rxmsg);
                break;
            default: break;
        }
    } else if (canp == &CAND2) {
        switch (rxmsg->SID) {
            case CAN_GIMBAL_BOARD_ID:
            case CAN_CHASSIS_BOARD_ID:
            case CAN_NVIDIA_TX2_BOARD_ID:
                can_process_communication(rxmsg);
                break;
            case CAN_UWB_MSG_ID:
                // can_process_uwb(rxmsg);
                break;
            default: break;
        }
    } else {
        // Throw error
    }
}

/*
 * Receiver threads
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_WORKING_AREA(can_rx2_wa, 256);

static THD_FUNCTION(can_rx, p)
{
    CANDriver *canp = (CANDriver *) p;
    chRegSetThreadName("can receiver");

    event_listener_t el;
    CANRxFrame rxmsg;
    chEvtRegister(&canp->rxfull_event, &el, 0);
    while (!chThdShouldTerminateX()) {
        if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
            continue;
        while (canReceive(canp, CAN_ANY_MAILBOX,
                          &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
            can_process_encoder_message(canp, &rxmsg);
        }
    }
    chEvtUnregister(&canp->rxfull_event, &el);
}

/*
 * Configure the CAN bus devices and start the two CAN bus thread
 * Receiver thread only. The commands are sent from chassis_init() and gimbal_init()
 */
void can_bus_init(void)
{
    uint8_t i;
    for (i = 0; i < CAN_FILTER_NUM; ++i) {
        canfilter[i].filter = i;
        canfilter[i].mode   = 0; // CAN_FilterMode_IdMask
        canfilter[i].scale  = 1; // CAN_FilterScale_32bit
        canfilter[i].assignment = 0;
        canfilter[i].register1  = 0;
        canfilter[i].register2  = 0;
    }

    canSTM32SetFilters(14, CAN_FILTER_NUM, canfilter);
//    canSTM32SetFilters(14, 0, canfilter);

    canStart(&CAND1, &cancfg);
    canStart(&CAND2, &cancfg);

    /*
     * Starting the transmitter and receiver threads
     */
    chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
                      can_rx, (void *) &CAND1);
    chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,
                      can_rx, (void *) &CAND2);

    chThdSleepMilliseconds(10);
}