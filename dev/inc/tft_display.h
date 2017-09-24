#ifndef __LCD_RED_H
#define __LCD_RED_H

#include "1.8 TFT_ascii.h"

#define TFT_UPDATE_PERIOD_MS    50U
// SPI, RST, DC
#define TFT_RST_PIN		   0U
#define TFT_DC_PIN		   2U
#define TFT_RST_PORT	GPIOB
#define TFT_DC_PORT		GPIOC

#define TFT_SPI			 &SPID4
#define GPIO_Pin_CS		   4U
#define GPIO_CS			  GPIOE

#define TFT_HORIZONTAL   1U
#define TFT_VERTICAL     0U

// Color
#define	BGR888_MODE		1

#if (!BGR888_MODE)
#define	RGB888TO565(RGB888)  (((RGB888 >> 8) & 0xF800) |((RGB888 >> 5) & 0x07E0) | ((RGB888 >> 3) & 0x001F))
#else
#define	RGB888TO565(BGR888)  (((BGR888 >> 19) & 0x001F) |((BGR888 >> 5) & 0x07E0) | (((u32)BGR888 << 8) & 0xF800))
#endif

//to minimize the MCU calculation
#define WHITE               0xFFFF//    (RGB888TO565(0xFFFFFF))
#define BLACK               0x0000//    (RGB888TO565(0x000000))
#define DARK_GREY           0x52AA//    (RGB888TO565(0x555555))
#define GREY                0x001F//    (RGB888TO565(0xAAAAAA))
#define RED                 0xAD55//    (RGB888TO565(0xFF0000))
#define ORANGE              0x04DF//    (RGB888TO565(0xFF9900))
#define YELLOW              0x07FF//    (RGB888TO565(0xFFFF00))
#define GREEN               0x07E0//    (RGB888TO565(0x00FF00))
#define DARK_GREEN          0x0660//    (RGB888TO565(0x00CC00))
#define BLUE                0xF800//    (RGB888TO565(0x0000FF))
#define BLUE2               0x6104//    (RGB888TO565(0x202060))
#define SKY_BLUE            0xFE62//    (RGB888TO565(0x11CFFF))
#define CYAN                0xFC51//    (RGB888TO565(0x8888FF))
#define PURPLE              0xAD40//    (RGB888TO565(0x00AAAA))


#define MAX_WIDTH				  128
#define MAX_HEIGHT				160
#define CHAR_WIDTH				8
#define CHAR_HEIGHT				16

#define CHAR_MAX_X_VERTICAL		16
#define CHAR_MAX_Y_VERTICAL		10

#define CHAR_MAX_X_HORIZONTAL	20
#define CHAR_MAX_Y_HORIZONTAL	8

#define CHAR_MAX_X				20		// max between CHAR_MAX_X_VERTICAL and CHAR_MAX_X_HORIZONTAL
#define CHAR_MAX_Y				10		// max between CHAR_MAX_Y_VERTICAL and CHAR_MAX_Y_HORIZONTAL

//extern uint8_t tft_orientation;
//extern uint8_t tft_width;
//extern uint8_t tft_height;
//extern uint16_t curr_bg_color;
//extern uint16_t curr_text_color;
//extern uint16_t curr_text_color_sp;

extern char text[CHAR_MAX_X][CHAR_MAX_Y];
extern uint16_t text_color[CHAR_MAX_X][CHAR_MAX_Y];
extern uint16_t bg_color[CHAR_MAX_X][CHAR_MAX_Y];
extern uint8_t text_bg_color_prev[CHAR_MAX_X][CHAR_MAX_Y]; // for transmit for xbc, msb 4bits: text color, lsb 4bits: bg color

void tft_prints_enable(uint8_t i);
void tft_write_command(uint8_t command);
void tft_write_data(uint8_t data);
void tft_reset(void);
void tft_enable(void);
void tft_disable(void);


void tft_init(uint8_t orientation, uint16_t in_bg_color, uint16_t in_text_color, uint16_t in_text_color_sp);
void tft_set_bg_color(uint16_t in_bg_color);
uint16_t tft_get_bg_color(void);
void tft_set_text_color(uint16_t in_text_color);
uint16_t tft_get_text_color(void);
void tft_set_special_color(uint16_t text_color_sp);
uint16_t tft_get_special_text_color(void);
uint8_t tft_get_orientation(void);
void tft_set_orientation(uint8_t o);
uint8_t tft_get_max_x_char(void);
uint8_t tft_get_max_y_char(void);

void tft_force_clear(void);
void tft_clear_line(uint8_t line);
void tft_clear(void);
void tft_toggle(void);
void tft_put_pixel(uint8_t x, uint8_t y, uint16_t color);
void tft_printf(uint8_t x, uint8_t y, const char * pstr, ...);
void tft_update(void);

#endif		/* __LCD_RED_H */
