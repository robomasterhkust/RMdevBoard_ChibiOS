#ifndef _DBUS_H_
#define _DBUS_H_

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN              ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024)
#define RC_CH_VALUE_MAX              ((uint16_t)1684)
#define DBUS_BUFFER_SIZE             ((uint8_t)18)

#define UART_DBUS                     &UARTD1

typedef union{
		struct{
			uint16_t channel0;
			uint16_t channel1;
			uint16_t channel2;
			uint16_t channel3;
			uint8_t  s1;
			uint8_t  s2;
		}rc;
}RC_Ctl_t;

RC_Ctl_t* RC_get(void);
void RC_init(void);

#endif
