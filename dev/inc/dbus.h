#ifndef _DBUS_H_
#define _DBUS_H_

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN              ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024)
#define RC_CH_VALUE_MAX              ((uint16_t)1684)
#define DBUS_BUFFER_SIZE             ((uint8_t)18)

#define UART_DBUS                     &UARTD1

#define KEY_V       0x4000
#define KEY_C       0x2000
#define KEY_X       0x1000
#define KEY_Z       0x0800
#define KEY_G       0x0400
#define KEY_F       0x0200
#define KEY_R       0x0100
#define KEY_E       0x0080
#define KEY_Q       0x0040
#define KEY_CTRL    0x0020
#define KEY_SHIFT   0x0010
#define KEY_D       0x0008
#define KEY_A       0x0004
#define KEY_S       0x0002
#define KEY_W       0x0001
typedef struct{
		struct{
			uint16_t channel0;
			uint16_t channel1;
			uint16_t channel2;
			uint16_t channel3;
			uint8_t  s1;
			uint8_t  s2;
		}rc;

		struct{
		  /*
		   * Range: [-32768,32767]
		   *
		   * */
		  int16_t x;
		  int16_t y;
		  int16_t z;
		  /*
		   * Range: [0 || 1]
		   * 0 : Not pressed
		   * 1:  Pressed
		   * */
		  uint8_t LEFT;
		  uint8_t RIGHT;
		}mouse;

		struct{
	        /*
	            Bitmap:
	                15  14  13  12  11  10  9   8   7   6   5   4   3   2   1
	                V   C   X   Z   G   F   R   E   Q CTR SHT   D   A   S   W
	        */
		  uint16_t key_code;
		}keyboard;
}RC_Ctl_t;

RC_Ctl_t* RC_get(void);
void RC_init(void);

#endif
