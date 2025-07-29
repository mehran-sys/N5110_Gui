#ifndef __PCD8544_DRIVER
#define __PCD8544_DRIVER

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "pcd8544_font.h"
#include "stm32f1xx_hal.h"

#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80
#define PCD8544_DISPLAY_BLANK 0x08
#define PCD8544_DISPLAY_NORMAL 0x0C
#define PCD8544_DISPLAY_ALL_ON 0x09
#define PCD8544_DISPLAY_INVERTED 0x0D

#define PCD8544_WIDTH 84
#define PCD8544_HEIGHT 48
#define PCD8544_SIZE PCD8544_WIDTH * PCD8544_HEIGHT / 8
#define PCD8544_WRAP_TEXT_PADDING 2

#define PCD8544_RSTPORT GPIOB
#define PCD8544_RSTPIN GPIO_PIN_0
#define PCD8544_CEPORT GPIOA
#define PCD8544_CEPIN GPIO_PIN_6
#define PCD8544_DCPORT GPIOA
#define PCD8544_DCPIN GPIO_PIN_4
#define PCD8544_DRIVER_SPI hspi1

#define PCD8544_DEFAULT_CONTRAST 60

extern SPI_HandleTypeDef hspi1;

typedef enum {
	ALIGNMETN_RIGHT,
	ALIGNMETN_LEFT,
	ALIGNMETN_CENTER
} Alignment_e;

void PCD8544_sendData(uint8_t val);
void PCD8544_command(uint8_t data);
void PCD8544_data(uint8_t data);
void PCD8544_init(uint8_t contrast);
void PCD8544_setInvert(bool mode);
void PCD8544_setTextInvert(bool mode);
void PCD8544_clearScreen();
void PCD8544_goXY(uint8_t x, uint8_t y);
void PCD8544_setFont(uint8_t font_number);
void PCD8544_setContrast(uint8_t contrast);
uint8_t PCD8544_getContrast();
void PCD8544_refreshScreen();
void PCD8544_refreshArea(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax);
void PCD8544_fillScreen();

void PCD8544_drawPixel(uint8_t x, uint8_t y, bool pixel);

void PCD8544_drawHorizontalLine(int x, int y, int l);
void PCD8544_drawVerticalLine(int x, int y, int l);
void PCD8544_drawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void PCD8544_drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void PCD8544_drawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void PCD8544_drawRoundRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t r);
void PCD8544_drawFilledRoundRectangle(int8_t x, int8_t y, int8_t w, int8_t h, int8_t r);
void PCD8544_drawCircle(int8_t x0, int8_t y0, int8_t r);
void PCD8544_fillCircle(uint8_t x0, uint8_t y0, uint8_t r);
void PCD8544_drawCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t cornername);
void PCD8544_drawFilledCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t corners, uint8_t delta);
void PCD8544_drawTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void PCD8544_drawBitmap(uint8_t x, uint8_t y, const uint8_t bitmap[],uint8_t w, uint8_t h);

void PCD8544_printTime(uint8_t hour, uint8_t minute);
void PCD8544_drawClockDeliminator();

void PCD8544_printChar(const uint8_t ch, uint8_t x, uint8_t y);
void PCD8544_printString(const char *str, uint8_t x, uint8_t y);
void PCD8544_printStringAlign(const char *str, uint8_t y, Alignment_e alignment);


#endif
