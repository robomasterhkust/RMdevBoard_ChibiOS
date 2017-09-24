#include "ch.h"
#include "hal.h"

#include "tft_display.h"
#include "chprintf.h"
#include "memstreams.h"

//private data
uint16_t curr_bg_color = BLACK;
uint16_t curr_text_color = BLACK;
uint16_t curr_text_color_sp = BLACK;

uint8_t tft_orientation = 0, tft_enabled = 1;


char text[CHAR_MAX_X][CHAR_MAX_Y];
char text_prev[CHAR_MAX_X][CHAR_MAX_Y];
uint16_t text_color[CHAR_MAX_X][CHAR_MAX_Y];
uint16_t text_color_prev[CHAR_MAX_X][CHAR_MAX_Y];
uint16_t bg_color[CHAR_MAX_X][CHAR_MAX_Y];
uint16_t bg_color_prev[CHAR_MAX_X][CHAR_MAX_Y];

uint16_t print_pos = 0;

uint8_t prints_enabled = 1;

void tft_fill_color(uint16_t color);

static const SPIConfig TFTSPI_cfg =
{
  NULL,
  GPIO_CS,
  GPIO_Pin_CS,
  SPI_CR1_MSTR |
  SPI_CR1_CPHA | SPI_CR1_CPOL
};

void tft_prints_enable(uint8_t i)
{
	prints_enabled = i;
}

/**
  * @brief  Sending a command
  * @param  command: one byte command to be sent
  * @retval None
  */
void tft_write_command(uint8_t command)
{
	palClearPad(TFT_DC_PORT, TFT_DC_PIN);

  spiSelect(TFT_SPI);
  spiSend(TFT_SPI, 1, &command);
  spiUnselect(TFT_SPI);
}

/**
  * @brief  Sending a data
  * @param  data: one byte data to be sent
  * @retval None
  */
void tft_write_data(uint8_t data)
{
	palSetPad(TFT_DC_PORT, TFT_DC_PIN);

  spiSelect(TFT_SPI);
  spiSend(TFT_SPI, 1, &data);
  spiUnselect(TFT_SPI);
}

/**
  * @brief  Configuration of TFT
  * @param  None
  * @retval None
  */
void tft_config(void)
{
	tft_write_command(0x01);   //Sofeware setting
	//chThdSleepMilliseconds(0);
	tft_write_command(0x11);//Sleep out
	chThdSleepMilliseconds(120);

	//ST7735R Frame Rate
	tft_write_command(0xB1);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_command(0xB2);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_command(0xB3);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	tft_write_data(0x01);
	tft_write_data(0x2C);
	tft_write_data(0x2D);
	//------------------------------------End ST7735R Frame Rate-----------------------------------------//
	tft_write_command(0xB4);//Column inversion
	tft_write_data(0x07);
	//------------------------------------ST7735R Power Sequence-----------------------------------------//
	tft_write_command(0xC0);
	tft_write_data(0xA2);
	tft_write_data(0x02);
	tft_write_data(0x84);
	tft_write_command(0xC1);
	tft_write_data(0xC5);
	tft_write_command(0xC2);
	tft_write_data(0x0A);
	tft_write_data(0x00);
	tft_write_command(0xC3);
	tft_write_data(0x8A);
	tft_write_data(0x2A);
	tft_write_command(0xC4);
	tft_write_data(0x8A);
	tft_write_data(0xEE);
	//---------------------------------End ST7735R Power Sequence-------------------------------------//
	tft_write_command(0xC5);//VCOM
	tft_write_data(0x0E);
	tft_write_command(0x36);//MX, MY, RGB mode
	tft_write_data(0xC8);
	//------------------------------------ST7735R Gamma Sequence-----------------------------------------//
	tft_write_command(0xe0);
	tft_write_data(0x02);
	tft_write_data(0x1c);
	tft_write_data(0x07);
	tft_write_data(0x12);
	tft_write_data(0x37);
	tft_write_data(0x32);
	tft_write_data(0x29);
	tft_write_data(0x2d);
	tft_write_data(0x29);
	tft_write_data(0x25);
	tft_write_data(0x2b);
	tft_write_data(0x39);
	tft_write_data(0x00);
	tft_write_data(0x01);
	tft_write_data(0x03);
	tft_write_data(0x10);
	tft_write_command(0xe1);
	tft_write_data(0x03);
	tft_write_data(0x1d);
	tft_write_data(0x07);
	tft_write_data(0x06);
	tft_write_data(0x2e);
	tft_write_data(0x2c);
	tft_write_data(0x29);
	tft_write_data(0x2d);
	tft_write_data(0x2e);
	tft_write_data(0x2e);
	tft_write_data(0x37);
	tft_write_data(0x3f);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x02);
	tft_write_data(0x10);
	tft_write_command(0x2A);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x7f);

	tft_write_command(0x2B);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x00);
	tft_write_data(0x9f);
	//------------------------------------End ST7735R Gamma Sequence-----------------------------------------//

	tft_write_command(0x3A);
	tft_write_data(0x05);
	tft_write_command(0x29);//Display on

	chThdSleepMilliseconds(10);
}

/**
  * @brief  Hardware reset for TFT
  * @param  None
  * @retval None
  */
void tft_reset(void)
{
 	palClearPad(TFT_RST_PORT, TFT_RST_PIN);
	chThdSleepMilliseconds(1);
	palSetPad(TFT_RST_PORT, TFT_RST_PIN);
	chThdSleepMilliseconds(1);
}

/**
  * @brief  Initialization of TFT
  * @param  None
  * @retval None
  */

/**
  * @brief  Enable using TFT
  * @param  None
  * @retval None
  */
void tft_enable(void)
{
	tft_enabled = 1;
}

/**
  * @brief  Disable using TFT
  * @param  None
  * @retval None
  */
void tft_disable(void)
{
	tft_enabled = 0;
}

/**
  * @brief  Set the current background color in RGB565
  * @param  None
  * @retval None
  */
void tft_set_bg_color(uint16_t in_bg_color)
{
	curr_bg_color = in_bg_color;
}

/**
	* @brief Get the current background color
	* @param None
	* @retval Current background color in RGB565
	*/
uint16_t tft_get_bg_color(void)
{
	return curr_bg_color;
}

/**
  * @brief  Set the current text color
  * @param  None
  * @retval None
  */
void tft_set_text_color(uint16_t in_text_color)
{
	curr_text_color = in_text_color;
}

/**
	* @brief Set the current text color in RGB565
	* @param None
	* @retval Current text color in RGB565
	*/
uint16_t tft_get_text_color(void)
{
	return curr_text_color;
}


/**
  * @brief  Set the current special text color
  * @param  None
  * @retval None
  */
void tft_set_special_color(uint16_t text_color_sp)
{
	curr_text_color_sp = text_color_sp;
}

/**
	* @brief Set the current text color in RGB565
	* @param None
	* @retval Current text color in RGB565
	*/
uint16_t tft_get_special_text_color(void)
{
	return curr_text_color_sp;
}

/**
  * @brief Get the current orientation of the TFT monitor
  * @param None
  * @retval The current orientation (0 - 3)
  */
uint8_t tft_get_orientation(void)
{
	return tft_orientation;
}

/**
  * @brief Set the orientation of the TFT monitor
  * @param o: orientation (0 - 3)
  * @retval None
  */
void tft_set_orientation(uint8_t o)
{
	tft_orientation = o % 4;
	tft_force_clear();
}

/**
  * @brief Get the max character per row exclusively (based on the current orientation)
  * @param None
  * @retval The maximum character per row exclusively (if n is returned, range of display is 0..n-1)
  */
uint8_t tft_get_max_x_char(void)
{
  return (tft_orientation % 2) ? CHAR_MAX_X_HORIZONTAL : CHAR_MAX_X_VERTICAL;
}

/**
  * @brief Get the max character per column exclusively (based on the current orientation)
  * @param None
  * @retval The maximum character per column exclusively (if n is returned, range of display is 0..n-1)
  */
uint8_t tft_get_max_y_char(void)
{
  return (tft_orientation % 2) ? CHAR_MAX_Y_HORIZONTAL : CHAR_MAX_Y_VERTICAL;
}

/**
  * @brief  Set the position of one pixel
  * @param  None
  * @retval None
  */
void tft_set_pixel_pos(uint8_t x, uint8_t y)
{
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(x); 				// X START
	tft_write_data(0x00);
	tft_write_data(x+1); 			// X END

	tft_write_command(0x2b);		// Row addr set
	tft_write_data(0x00);
	tft_write_data(y);				// Y START
	tft_write_data(0x00);
	tft_write_data(y+1);			// Y END

	tft_write_command(0x2c); 		// write to RAM
}

/**
  * @brief  Set the position of some characters
  * @param  None
  * @retval None
  */
void tft_set_char_pos(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(x1); 			//X START
	tft_write_data(0x00);
	tft_write_data(x2); 			//X END

	tft_write_command(0x2b);		//Row addr set
	tft_write_data(0x00);
	tft_write_data(y1);			//Y START
	tft_write_data(0x00);
	tft_write_data(y2);		//Y END

	tft_write_command(0x2c); 		// write to RAM
}


/**
  * @brief  Fill the whole screen with a color
  * @param  color: color to be filled with
  * @retval None
  */
void tft_fill_color(uint16_t color)
{
	uint16_t i;					//160*128

	tft_write_command(0x2a);		// Column addr set
	tft_write_data(0x00);
	tft_write_data(0x00); 				// X START
	tft_write_data(0x00);
	tft_write_data(0x7f); 			// X END

	tft_write_command(0x2b);		// Row addr set
	tft_write_data(0x00);
	tft_write_data(0x00);				// Y START
	tft_write_data(0x00);
	tft_write_data(0x9f);			// Y END

	tft_write_command(0x2c); 		// write to RAM

  //spiSelect(TFT_SPI);
	for (i = 0; i < MAX_WIDTH*MAX_HEIGHT; i++) {
		tft_write_data(color >> 8);
		tft_write_data(color);
	}
  //spiUnselect(TFT_SPI);
}

/**
  * @brief  Clear every piexl on screen
  * @param  None
  * @retval None
  */
void tft_force_clear(void)
{
	uint8_t x, y;
	for (x = 0; x < CHAR_MAX_X; x++) {
		for (y = 0; y < CHAR_MAX_Y; y++) {
			text_prev[x][y] = ' ';
			text_color_prev[x][y] = curr_text_color;
			bg_color_prev[x][y] = curr_bg_color;
		}
	}
	tft_fill_color(curr_bg_color);
}

/**
  * @brief  Clear one line on screen
  * @param  line: the line to be cleared
  * @retval None
  */
void tft_clear_line(uint8_t line)
{
	uint8_t x;
	for (x = 0; x < CHAR_MAX_X; x++) {
		text[x][line] = ' ';
		text_color[x][line] = curr_text_color;
		bg_color[x][line] = curr_bg_color;
	}
}

/**
  * @brief  Clear all lines on screen
  * @param  None
  * @retval None
  */
void tft_clear(void)
{
	uint8_t y;
	for(y = 0; y < CHAR_MAX_Y; y++)
		tft_clear_line(y);
}

/**
  * @brief  Switch the orientation of screen
  * @param  None
  * @retval None
  */
void tft_toggle(void)
{
	tft_force_clear();
	tft_clear();
	tft_orientation = (tft_orientation+1) % 4;
}

/**
  * @brief  Print a single pixel on screen
  * @param  x: x-coordinate
  * @param  y: y-coordinate
  * @param  color: color of the pixel
  * @retval None
  */
void tft_put_pixel(uint8_t x, uint8_t y, uint16_t color)
{
	switch (tft_orientation) {
		case 0:
			tft_set_pixel_pos(x, y);
			break;
		case 1:
			tft_set_pixel_pos(MAX_WIDTH-y-1, x);
			break;
		case 2:
			tft_set_pixel_pos(MAX_WIDTH-x-1, MAX_HEIGHT-y-1);
			break;
		case 3:
			tft_set_pixel_pos(y, MAX_HEIGHT-x-1);
			break;
	}
    tft_write_data(color >> 8);
    tft_write_data(color);
}

uint8_t tft_char_is_changed(uint8_t x, uint8_t y)
{
	uint8_t re = (text_prev[x][y] != text[x][y] || text_color_prev[x][y] != text_color[x][y] || bg_color_prev[x][y] != bg_color[x][y]);
	text_prev[x][y] = text[x][y];
	text_color_prev[x][y] = text_color[x][y];
	bg_color_prev[x][y] = bg_color[x][y];
	return re;
}

/**
  * @brief  Print a string at certain position ("[...]" as special-color character)
  * @param  x: starting x-coordinate
  * @param  y: starting y-coordinate
  * @param  pstr: string to be printed
  * @retval None
  */
void tft_printf(uint8_t x, uint8_t y, const char * pstr, ...)
{
	uint8_t buf[256], is_special = 0;
	uint8_t* fp = NULL;
	va_list arglist;

  MemoryStream ms;
  msObjectInit(&ms, buf, 255, 0);

  va_start(arglist, pstr);
  chvprintf((BaseSequentialStream *)(void *)&ms, pstr, arglist);
  va_end(arglist);

  if (ms.eos < 255)
      buf[ms.eos] = 0;

  fp = buf;

	while (*fp)	{
		if (*fp == '[' && *(fp - 1) != '\\') {
			is_special = 1;
			fp++;
		} else if (*fp == ']' && *(fp - 1) != '\\') {
			is_special = 0;
			fp++;
		} else if (*fp == '\r' || *fp == '\n') {
			fp++;
		} else {
			if (x > CHAR_MAX_X || y > CHAR_MAX_Y) {
				fp++;
				continue;
			}
      if (*fp == '\\' && (*(fp+1) == '[' || *(fp+1) == ']')) {
        fp++;
      }
			text[x][y] = *fp++;
			text_color[x][y] = is_special ? curr_text_color_sp : curr_text_color;
			bg_color[x][y] = curr_bg_color;
			if (x >= CHAR_MAX_X) {
				x = 0;
				y++;
			} else {
				x++;
			}
			if (y >= CHAR_MAX_Y)
				y = 0;
		}
	}
}

/**
  * @brief  Refresh the whole screen
  * @param  None
  * @retval None
  */
void tft_update(void)
{
	int16_t x, y, x2, y2, px, py;
	int16_t char_n = 0;
	uint16_t clr;

	if (!tft_enabled)
		return;

	switch (tft_orientation) {
		case 0:
			for (y = 0; y < CHAR_MAX_Y_VERTICAL; y++) {
				for (x = 0; x < CHAR_MAX_X_VERTICAL; x++) {
					if (tft_char_is_changed(x, y)) {
						char_n = 1;
						while (x+char_n < CHAR_MAX_X_VERTICAL && tft_char_is_changed(x+char_n, y)) {
							if (x+char_n >= CHAR_MAX_X_VERTICAL) {
								break;
							} else {
								char_n++;
							}
						}
						tft_set_char_pos(x*CHAR_WIDTH, y*CHAR_HEIGHT, (x+char_n)*CHAR_WIDTH-1, (y+1)*CHAR_HEIGHT-1);
						y2 = y;

            for (py = 0; py < CHAR_HEIGHT; py++) {
							for (px = 0; px < char_n*CHAR_WIDTH; px++) {
								x2 = x+px/CHAR_WIDTH;
								clr = ascii_8x16[((text[x2][y2] - STARTING_ASCII) * CHAR_HEIGHT) + py] & (0x80 >> (px % CHAR_WIDTH)) ? text_color[x2][y2] : bg_color[x2][y2];
								tft_write_data(clr >> 8);
								tft_write_data(clr);
							}
						}

						x += char_n-1;
					}
				}
			}
			break;
		case 1:
			for (x = 0; x < CHAR_MAX_X_HORIZONTAL; x++) {
				for (y = CHAR_MAX_Y_HORIZONTAL-1; y >= 0; y--) {
					if (tft_char_is_changed(x, y)) {
						char_n = 1;
						while (y-char_n > -1 && tft_char_is_changed(x, y-char_n)) {
							if (y-char_n <= -1) {
								break;
							} else {
								char_n++;
							}
						}
						tft_set_char_pos((CHAR_MAX_Y_HORIZONTAL-y-1)*CHAR_HEIGHT, x*CHAR_WIDTH, (CHAR_MAX_Y_HORIZONTAL-y-1+char_n)*CHAR_HEIGHT-1, (x+1)*CHAR_WIDTH-1);
						x2 = x;

            for (px = 0; px < CHAR_WIDTH; px++) {
							for (py = 0; py < char_n*CHAR_HEIGHT; py++) {
								y2 = y-py/CHAR_HEIGHT;
								clr = ascii_8x16[((text[x2][y2] - STARTING_ASCII) * CHAR_HEIGHT) + CHAR_HEIGHT-(py % CHAR_HEIGHT)-1] & (0x80 >> px) ? text_color[x2][y2] : bg_color[x2][y2];
								tft_write_data(clr >> 8);
								tft_write_data(clr);
							}
						}

						y -= char_n-1;
					}
				}
			}
			break;
		case 2:
			for (y = CHAR_MAX_Y_VERTICAL-1; y >= 0; y--) {
				for (x = CHAR_MAX_X_VERTICAL-1; x >= 0; x--) {
					if (tft_char_is_changed(x, y)) {
						char_n = 1;
						while (x-char_n > -1 && tft_char_is_changed(x-char_n, y)) {
							if (x-char_n <= -1) {
								break;
							} else {
								char_n++;
							}
						}
						tft_set_char_pos((CHAR_MAX_X_VERTICAL-x-1)*CHAR_WIDTH, (CHAR_MAX_Y_VERTICAL-y-1)*CHAR_HEIGHT, (CHAR_MAX_X_VERTICAL-x-1+char_n)*CHAR_WIDTH-1, (CHAR_MAX_Y_VERTICAL-y)*CHAR_HEIGHT-1);
						y2 = y;

            for (py = 0; py < CHAR_HEIGHT; py++) {
							for (px = 0; px < char_n*CHAR_WIDTH; px++) {
								x2 = x-px/CHAR_WIDTH;
								clr = ascii_8x16[((text[x2][y2] - STARTING_ASCII) * CHAR_HEIGHT) + (CHAR_HEIGHT-py-1)] & (0x80 >> (CHAR_WIDTH-(px % CHAR_WIDTH)-1)) ? text_color[x2][y2] : bg_color[x2][y2];
								tft_write_data(clr >> 8);
								tft_write_data(clr);
							}
						}

						x -= char_n-1;
					}
				}
			}
			break;
		case 3:
			for (x = CHAR_MAX_X_HORIZONTAL-1; x >= 0; x--) {
				for (y = 0; y < CHAR_MAX_Y_HORIZONTAL; y++) {
					if (tft_char_is_changed(x, y)) {
						char_n = 1;
						while (y+char_n < CHAR_MAX_Y_HORIZONTAL && tft_char_is_changed(x, y+char_n)) {
							if (y+char_n >= CHAR_MAX_Y_HORIZONTAL) {
								break;
							} else {
								char_n++;
							}
						}
						tft_set_char_pos(y*CHAR_HEIGHT, (CHAR_MAX_X_HORIZONTAL-x-1)*CHAR_WIDTH, (y+char_n)*CHAR_HEIGHT-1, (CHAR_MAX_X_HORIZONTAL-x)*CHAR_WIDTH-1);
						x2 = x;

            for (px = 0; px < CHAR_WIDTH; px++) {
							for (py = 0; py < char_n*CHAR_HEIGHT; py++) {
								y2 = y+py/CHAR_HEIGHT;
								clr = ascii_8x16[((text[x2][y2] - STARTING_ASCII) * CHAR_HEIGHT) + (py % CHAR_HEIGHT)] & (0x80 >> (CHAR_WIDTH-px-1)) ? text_color[x2][y2] : bg_color[x2][y2];
								tft_write_data(clr >> 8);
								tft_write_data(clr);
							}
						}

						y += char_n-1;
					}
				}
			}
			break;
	}
}

static THD_WORKING_AREA(TFT_thread_wa, 4096);
static THD_FUNCTION(TFT_thread, p)
{
  (void)p;
  chRegSetThreadName("TFT Display");

  while(true)
  {
    tft_update();
    chThdSleepMilliseconds(TFT_UPDATE_PERIOD_MS);
  }
}

void tft_init(uint8_t orientation, uint16_t in_bg_color,
  uint16_t in_text_color, uint16_t in_text_color_sp)
{
  palSetPad(GPIO_CS, GPIO_Pin_CS);
  spiAcquireBus(TFT_SPI);
  spiStart(TFT_SPI, &TFTSPI_cfg);

  tft_reset();
  tft_config();
  tft_write_command(0x2C);

  tft_set_bg_color(in_bg_color);
	tft_set_text_color(in_text_color);
	tft_set_special_color(in_text_color_sp);
	tft_fill_color(in_bg_color);
	tft_orientation = orientation;

  uint16_t x,y;
	for (x = 0; x < CHAR_MAX_X; x++) {
		for (y = 0; y < CHAR_MAX_Y; y++) {
			text[x][y] = ' ';
			text_color[x][y] = in_text_color;
			bg_color[x][y] = in_bg_color;

			text_prev[x][y] = ' ';
			text_color_prev[x][y] = in_text_color;
			bg_color_prev[x][y] = in_bg_color;
		}
	}

  chThdCreateStatic(TFT_thread_wa, sizeof(TFT_thread_wa),
  NORMALPRIO - 10,
                    TFT_thread, NULL);
}
