#ifndef _MAVLINK_CONF_H_
#define _MAVLINK_CONF_H_

/* ENTERPRIZE MAVlink transeiver V1.0 (Prototype)
  Use instruction:
    Read message from topic:
      mavlink_type_t* message = mavlinkComm_type_subscribe(void);

      chSysLock();
      data0 = message->data0;
      data1 = message->data1;
      ...
      chSysUnlock();

    Write message to transmit topic:

      mavlink_type_t message;
      mavlinkComm_type_publish(&message, transmit_freq);

      chSysLock();
      message.data0 = data0;
      message.data1 = data1;
      ...
      chSysUnlock();

    Develop instruction:
      1. import new mavlink message type header file
      2. modify mavlink_comm.h and mavlink_topic.c individually and ONLY
*/

/*  Define which topic you need here*/

#define MAVLINK_USE_HEARTBEAT       TRUE

#define MAVLINK_USE_ATTITUDE        FALSE

/**/

#include "mavlink_types.h"

#define UART_MAVLINK &UARTD3            //MAVLINK_COMM_0
//#define SERIAL_MAVLINK &SD3
#define UART_MAVLINK_BR 115200

#define MAVLINK_COMM_TEST

typedef struct{
  systime_t start_time;
  mavlink_status_t status;
  mavlink_message_t rx_message;

#ifdef MAVLINK_COMM_TEST
  systime_t end_time; //Used in loop test to determine transmission delay
#endif
} mavlinkComm_t;

mavlinkComm_t* mavlinkComm_get(void);
void mavlinkComm_init(void);

#ifdef MAVLINK_COMM_TEST
  void mavlinkComm_test(void);
#endif

typedef enum{
  MAVLINK_COMM_PUBLISH_OK = 0,
  MAVLINK_COMM_PUBLISH_BANDWIDTH_OVERLOAD,
  MAVLINK_COMM_PUBLISH_FULL
} mavlinkComm_publish_result_t;

#if (MAVLINK_USE_HEARTBEAT == TRUE) && !defined(__DOXYGEN__)
  #include "minimal/mavlink_msg_heartbeat.h"
  mavlink_heartbeat_t* mavlinkComm_heartbeat_subscribe(void);
  mavlinkComm_publish_result_t mavlinkComm_heartbeat_publish(mavlink_heartbeat_t* const message,
    const uint16_t transmit_freq);
#endif

/* This function is private*/ // Beck: linking issue, change to public
void _mavlinkComm_topic_decode(mavlink_message_t* const message);
void mavlinkComm_test(void);
#endif
