#include "ch.h"
#include "hal.h"

#include "minimal/mavlink.h"
//#include "mavlink_helpers.h"
#include "mavlink_comm.h"

mavlinkComm_t comm;
static thread_reference_t mavlink_receive_thread_handler = NULL;
static uint8_t rxbuf[MAVLINK_MAX_PACKET_LEN];

mavlinkComm_t* mavlinkComm_get(void)
{
  return &comm;
}

static void mavlinkComm_rxchar(void)
{
  USART_TypeDef *u = (*UART_MAVLINK).usart;
  uint32_t cr1 = u->CR1;

  if(u->DR == MAVLINK_STX
    || rxbuf[0] == MAVLINK_STX
  )
  {
    u->CR1 = cr1 & (~USART_CR1_RXNEIE);
    comm.start_time = chVTGetSystemTimeX();
    comm.status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;

    chSysLockFromISR();
    chThdResumeI(&mavlink_receive_thread_handler,MSG_OK);
    chSysUnlockFromISR();
  }
}

CH_IRQ_HANDLER(STM32_USART3_HANDLER) {

  CH_IRQ_PROLOGUE();

  mavlinkComm_rxchar();

  CH_IRQ_EPILOGUE();
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,NULL,NULL,NULL,NULL,
  UART_MAVLINK_BR,
  0,
  0,
  0
};

static THD_WORKING_AREA(mavlink_rx_wa, 2048);
static THD_FUNCTION(mavlink_rx, p)
{

  (void)p;
  chRegSetThreadName("Mavlink receiver");

  mavlink_message_t rx_message;
  mavlink_status_t rx_status;

  memset(&rx_message, 0, sizeof(mavlink_message_t));
  memset(&rx_status, 0, sizeof(mavlink_status_t));

  while(!chThdShouldTerminateX())
  {
    chSysLock();
    chThdSuspendS(&mavlink_receive_thread_handler);
    chSysUnlock();

    while(!rxbuf[1])
      chThdSleepMicroseconds(100);

    if(rxbuf[1] > MAVLINK_MAX_PAYLOAD_LEN)
      rxbuf[1] = MAVLINK_MAX_PAYLOAD_LEN;

    systime_t end_time = comm.start_time + US2ST((rxbuf[1] + 8)*1e7/UART_MAVLINK_BR) + 20;
    /*
      Transimission time estimation: bytes to transfer *
      (8bits per byte + 1 start bit + 1 stop bit) on UART / UART baudrate + 20us in case we missed the last bit
    */

    if(end_time > chVTGetSystemTimeX())
      chThdSleepUntil(end_time);

    uint8_t i = 0;
    do{
      comm.status.msg_received = mavlink_frame_char_buffer(&rx_message,
                                                        &rx_status,
                                                        rxbuf[i++],
                                                        &comm.rx_message,
                                                        &comm.status);
    }while(comm.status.msg_received == MAVLINK_FRAMING_INCOMPLETE);

    uartStopReceive(UART_MAVLINK);
    uartStartReceive(UART_MAVLINK, MAVLINK_MAX_PACKET_LEN, rxbuf);
    (*UART_MAVLINK).usart->CR1 |= USART_CR1_RXNEIE;
    rxbuf[0] = rxbuf[1] = 0;

    if(comm.status.msg_received == MAVLINK_FRAMING_OK)
      _mavlinkComm_topic_decode(&comm.rx_message);

    #ifdef MAVLINK_COMM_TEST
      comm.end_time = chVTGetSystemTimeX();
    #endif
  }
}

void mavlinkComm_init(void)
{
  memset(&comm,0,sizeof(mavlinkComm_t));

  uartStart(UART_MAVLINK, &uart_cfg);

  uartStopReceive(UART_MAVLINK);
  uartStartReceive(UART_MAVLINK, MAVLINK_MAX_PACKET_LEN, rxbuf);
  (*UART_MAVLINK).usart->CR1 |= USART_CR1_RXNEIE;

  chThdCreateStatic(mavlink_rx_wa, sizeof(mavlink_rx_wa), NORMALPRIO, mavlink_rx ,NULL);
}

#ifdef MAVLINK_COMM_TEST
void mavlinkComm_test(void)
{
  mavlink_heartbeat_t packet_test = {
    963497464,
    17,
    84,
    151,
    218,
    3
  };

  mavlink_message_t message;
  const uint16_t buf_len = MAVLINK_NUM_NON_PAYLOAD_BYTES + message.len;
  uint8_t buf[buf_len];

  mavlink_msg_heartbeat_pack(0, 200, &message,
          packet_test.type,
          packet_test.autopilot,
          packet_test.base_mode,
          packet_test.custom_mode,
          packet_test.system_status);

  mavlink_msg_to_send_buffer(buf, &message);
  uartStartSend(UART_MAVLINK, buf_len, buf);
}
#endif
