#include "pcd8544_driver.h"

uint8_t buffer[PCD8544_SIZE];

bool inverttext;
uint8_t _contrast;
uint8_t size_x;
uint8_t size_y;
uint8_t font = 1;

void PCD8544_sendData(uint8_t val){
    HAL_SPI_Transmit(&PCD8544_DRIVER_SPI, &val, 1, 10);
}

void PCD8544_command(uint8_t data){
	HAL_GPIO_WritePin(PCD8544_DCPORT, PCD8544_DCPIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PCD8544_CEPORT, PCD8544_CEPIN, GPIO_PIN_RESET);
	PCD8544_sendData(data);
	HAL_GPIO_WritePin(PCD8544_CEPORT, PCD8544_CEPIN, GPIO_PIN_SET);
}

void PCD8544_data(uint8_t data) {
	HAL_GPIO_WritePin(PCD8544_DCPORT, PCD8544_DCPIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PCD8544_CEPORT, PCD8544_CEPIN, GPIO_PIN_RESET);
	PCD8544_sendData(data);
	HAL_GPIO_WritePin(PCD8544_CEPORT, PCD8544_CEPIN, GPIO_PIN_SET);
}

void PCD8544_init(uint8_t contrast){
  HAL_GPIO_WritePin(PCD8544_CEPORT, PCD8544_CEPIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PCD8544_RSTPORT, PCD8544_RSTPIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(PCD8544_RSTPORT, PCD8544_RSTPIN, GPIO_PIN_SET);
  HAL_Delay(10);
  PCD8544_command(0x21); //LCD extended commands.
  PCD8544_command(0x80 | contrast); //set LCD Vop(Contrast).
  PCD8544_command(0x04); //set temp coefficent.
  PCD8544_command(0x13); //LCD bias mode 1:40.
  PCD8544_command(0x20); //LCD basic commands.
  PCD8544_command(PCD8544_DISPLAY_NORMAL); //LCD normal.
  PCD8544_clearScreen();
  PCD8544_refreshScreen();
  inverttext = false;
  _contrast = contrast;
}


void PCD8544_setInvert(bool mode){
  if(mode == true){
    PCD8544_command(PCD8544_DISPLAY_INVERTED);
  }
  else{
    PCD8544_command(PCD8544_DISPLAY_NORMAL);
  }
}

void PCD8544_setContrast(uint8_t contrast){
	if (contrast > 0x7f){
		contrast = 0x7f;
	}
	_contrast = contrast;
	PCD8544_command(0x21);
	PCD8544_command(0x80 | contrast);
	PCD8544_command(0x20);
}
/*
 * @brief Invert the colour of any text sent to the display
 * @param mode: true = inverted / false = normal
 */
void PCD8544_setTextInvert(bool mode){
  if(mode == true){
    inverttext = true;
  }
  else{
    inverttext = false;
  }
}

void PCD8544_setFont(uint8_t font_number){
	font = font_number;

	if (font == 1){
		size_x = sizeof(font1[0])/sizeof(font1[0][0]);
		size_y = sizeof(font1[0][0])*8;
	}
	else if (font == 2){
		size_x = sizeof(font2[0])/sizeof(font2[0][0]);
		size_y = sizeof(font2[0][0])*8;
	}
	else if (font == 3){
		size_x = sizeof(font3[0])/sizeof(font3[0][0]);
		size_y = sizeof(font3[0][0])*8;
	}
	else if (font == 4){
		size_x = sizeof(font4[0])/sizeof(font4[0][0]);
		size_y = sizeof(font4[0][0])*8;
	}
	else if (font == 5){
		size_x = sizeof(font5[0])/sizeof(font5[0][0]);
		size_y = sizeof(font5[0][0])*8;
	}
	else if (font == 6){
		size_x = sizeof(font6[0])/sizeof(font6[0][0]);
		size_y = sizeof(font6[0][0])*8;
	}
	else if (font == 7){
		size_x = sizeof(font7[0])/sizeof(font7[0][0]);
		size_y = sizeof(font7[0][0])*8;
	}
	else {
		size_x = sizeof(font2[0])/sizeof(font2[0][0]);
		size_y = sizeof(font2[0][0])*8;
	}
}

/*
 * @brief Puts one char on the current position of LCD's cursor
 * @param c: char to be printed
 */

void PCD8544_printChar(const uint8_t ch, uint8_t x, uint8_t y) {
	uint32_t column; // temp byte to store character's column bitmap
	if (x > PCD8544_WIDTH-size_x){
		x = PCD8544_WRAP_TEXT_PADDING;
	}

	for (int i = 0 ; i < size_x ; i++){
		if (font == 1)  column = font1[ch - 0x20][i];
		else if (font == 2) column = font2[ch - 0x20][i];
		else if (font == 3) column = font3[ch - 0x20][i];
		else if (font == 4) column = font4[ch - 0x20][i];
		else if (font == 5) column = font5[ch - 0x20][i];
		else if (font == 6) column = font6[ch - 0x2e][i];
		else if (font == 7) column = font7[ch - 0x2e][i];
		else column = font1[ch - 0x20][i];

		if (inverttext == true){
			column = ~column;
		}

	  for (int j = 0; j < size_y; j++){
	    if (column & (0x01 << j)) // test bits to set pixels
	      PCD8544_drawPixel(x+i, y+j, 1);
	    else
	      PCD8544_drawPixel(x+i, y+j, 0);
	  }
	}
}

/*
 * @brief Print a string on the LCD
 * @param x: starting point on the x-axis (column)
 * @param y: starting point on the y-axis (line)
 */
void PCD8544_printString(const char *str, uint8_t x, uint8_t y){
  while(*str){
   PCD8544_printChar(*str++,x,y);
	 x += size_x;
  }
}

void PCD8544_printStringAlign(const char *str, uint8_t y, Alignment_e alignment){
	uint8_t startX;

	if (alignment == ALIGNMETN_RIGHT){
		startX = PCD8544_WIDTH-(strlen(str)*size_x);
	}
	else if (alignment == ALIGNMETN_CENTER){
		startX = ((PCD8544_WIDTH)-(strlen(str)*size_x))/2;;
	}
	else if (alignment == ALIGNMETN_LEFT){
		startX = 0;
	}
	else {
		return;
	}

  while(*str){
   PCD8544_printChar(*str++,startX,y);
   startX += size_x;
   if (startX > 83){
  	 startX = 0;
  	 y += size_y;
   }
  }
}
/*
 * @brief Clear the screen
 */
void PCD8544_clearScreen(){
  for(int i = 0; i < 504; i++){
  	buffer[i] = 0;
  }
}

/*
 * @brief Set LCD's cursor to position X,Y
 * @param x: position on the x-axis (column)
 * @param y: position on the y-axis (line)
 */
void PCD8544_goXY(uint8_t x, uint8_t y){
  PCD8544_command(0x80 | x); //Column.
  PCD8544_command(0x40 | y); //Row.
}

/*
 * @brief Updates the entire screen according to lcd.buffer
 */
void PCD8544_refreshScreen(){
  PCD8544_goXY(PCD8544_SETXADDR, PCD8544_SETYADDR);
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < PCD8544_WIDTH; j++){
      PCD8544_data(buffer[(i * PCD8544_WIDTH) + j]);
    }
  }
}

/*
 * @brief Updates a square of the screen according to given values
 * @param xmin: starting point on the x-axis
 * @param xmax: ending point on the x-axis
 * @param ymin: starting point on the y-axis
 * @param ymax: ending point on the y-axis
 */
void PCD8544_refreshArea(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax){
  for(int i = 0; i < 6; i++){
    if(i * 8 > ymax){
      break;
    }
    //PCD8544_goXY(xmin, i);
    PCD8544_command(PCD8544_SETYADDR | i);
    PCD8544_command(PCD8544_SETXADDR | xmin);
    for(int j = xmin; j <= xmax; j++){
      PCD8544_data(buffer[(i * PCD8544_WIDTH) + j]);
    }
  }
}

/*
 * @brief Sets a pixel on the screen
 */
void PCD8544_drawPixel(uint8_t x, uint8_t y, bool pixel){
  if(x >= PCD8544_WIDTH || y >= PCD8544_HEIGHT){
    return;
  }
    
  if(pixel != false){
    buffer[x + (y / 8) * PCD8544_WIDTH] |= 1 << (y % 8);
  }
  else{
    buffer[x + (y / 8) * PCD8544_WIDTH] &= ~(1 << (y % 8));
  }
}

/*
 * @brief Draws a horizontal line
 * @param x: starting point on the x-axis
 * @param y: starting point on the y-axis
 * @param l: length of the line
 */
void PCD8544_drawHorizontalLine(int x, int y, int l){
  int by, bi;

  if ((x>=0) && (x<PCD8544_WIDTH) && (y>=0) && (y<PCD8544_HEIGHT)){
    for (int cx=0; cx<l; cx++){
      by=((y/8)*84)+x;
      bi=y % 8;
      buffer[by+cx] |= (1<<bi);
    }
  }
}

/*
 * @brief Draws a vertical line
 * @param x: starting point on the x-axis
 * @param y: starting point on the y-axis
 * @param l: length of the line
 */
void PCD8544_drawVerticalLine(int x, int y, int l){

  if ((x>=0) && (x<84) && (y>=0) && (y<48)){
    for (int cy=0; cy<= l; cy++){
      PCD8544_drawPixel(x, y+cy, true);
    }
  }
}

/*
 * @brief abs function used in PCD8544_drawLine
 * @param x: any integer
 * @return absolute value of x
 */
int abs(int x){
	if(x < 0){
		return x*(-1);
	}
	return x;
}

/*
 * @brief Draws any line
 * @param x1: starting point on the x-axis
 * @param y1: starting point on the y-axis
 * @param x2: ending point on the x-axis
 * @param y2: ending point on the y-axis
 */
void PCD8544_drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1){
	uint8_t tmp;
	int8_t steep = abs(y1 - y0) > abs(x1 - x0);
	  if (steep) {
	    tmp = x0;
	    x0 = y0;
	    y0 = tmp;
	    tmp = x1;
	    x1 = y1;
	    y1 = tmp;
	  }

	  if (x0 > x1) {
	    tmp = x0;
	    x0 = x1;
	    x1 = tmp;
	    tmp = y0;
	    y0 = y1;
	    y1 = tmp;
	  }

	  int8_t dx, dy;
	  dx = x1 - x0;
	  dy = abs(y1 - y0);

	  int8_t err = dx / 2;
	  int8_t ystep;

	  if (y0 < y1) {
	    ystep = 1;
	  } else {
	    ystep = -1;
	  }

	  for (; x0 <= x1; x0++) {
	    if (steep) {
	      PCD8544_drawPixel(y0, x0, 1);
	    } else {
	      PCD8544_drawPixel(x0, y0, 1);
	    }
	    err -= dy;
	    if (err < 0) {
	      y0 += ystep;
	      err += dx;
	    }
	}
}

/*
 * @brief Draws a rectangle
 * @param x1: starting point on the x-axis
 * @param y1: starting point on the y-axis
 * @param x2: ending point on the x-axis
 * @param y2: ending point on the y-axis
 */
void PCD8544_drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2){
  PCD8544_drawHorizontalLine(x1, y1, x2-x1);
  PCD8544_drawVerticalLine(x1, y1, y2-y1);
  PCD8544_drawVerticalLine(x2, y1, y2-y1);
  PCD8544_drawHorizontalLine(x1, y2, x2-x1);
}

void PCD8544_drawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h){
  for (uint8_t i = y ; i <= y+h ; i++){
	  PCD8544_drawHorizontalLine(x, i, w);
  }
}

void PCD8544_fillScreen(){
	PCD8544_drawFilledRectangle(0, 0, PCD8544_WIDTH, PCD8544_HEIGHT);
}

void PCD8544_drawCircle(int8_t x0, int8_t y0, int8_t r) {

  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  PCD8544_drawPixel(x0, y0 + r, 1);
  PCD8544_drawPixel(x0, y0 - r, 1);
  PCD8544_drawPixel(x0 + r, y0, 1);
  PCD8544_drawPixel(x0 - r, y0, 1);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    PCD8544_drawPixel(x0 + x, y0 + y, 1);
    PCD8544_drawPixel(x0 - x, y0 + y, 1);
    PCD8544_drawPixel(x0 + x, y0 - y, 1);
    PCD8544_drawPixel(x0 - x, y0 - y, 1);
    PCD8544_drawPixel(x0 + y, y0 + x, 1);
    PCD8544_drawPixel(x0 - y, y0 + x, 1);
    PCD8544_drawPixel(x0 + y, y0 - x, 1);
    PCD8544_drawPixel(x0 - y, y0 - x, 1);
  }
}

void PCD8544_drawCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t cornername) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4) {
      PCD8544_drawPixel(x0 + x, y0 + y, 1);
      PCD8544_drawPixel(x0 + y, y0 + x, 1);
    }
    if (cornername & 0x2) {
      PCD8544_drawPixel(x0 + x, y0 - y, 1);
      PCD8544_drawPixel(x0 + y, y0 - x, 1);
    }
    if (cornername & 0x8) {
      PCD8544_drawPixel(x0 - y, y0 + x, 1);
      PCD8544_drawPixel(x0 - x, y0 + y, 1);
    }
    if (cornername & 0x1) {
      PCD8544_drawPixel(x0 - y, y0 - x, 1);
      PCD8544_drawPixel(x0 - x, y0 - y, 1);
    }
  }
}

void PCD8544_fillCircle(uint8_t x0, uint8_t y0, uint8_t r) {
  PCD8544_drawVerticalLine(x0, y0 - r, 2 * r + 1);
  PCD8544_drawFilledCircleHelper(x0, y0, r, 3, 0);
}

void PCD8544_drawFilledCircleHelper(uint8_t x0, uint8_t y0, uint8_t r, uint8_t corners, uint8_t delta) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  delta++; // Avoid some +1's in the loop

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    // These checks avoid double-drawing certain lines, important
    // for the SSD1306 library which has an INVERT drawing mode.
    if (x < (y + 1)) {
      if (corners & 1)
        PCD8544_drawVerticalLine(x0 + x, y0 - y, 2 * y + delta);
      if (corners & 2)
    	PCD8544_drawVerticalLine(x0 - x, y0 - y, 2 * y + delta);
    }
    if (y != py) {
      if (corners & 1)
        PCD8544_drawVerticalLine(x0 + py, y0 - px, 2 * px + delta);
      if (corners & 2)
        PCD8544_drawVerticalLine(x0 - py, y0 - px, 2 * px + delta);
      py = y;
    }
    px = x;
  }
}

void PCD8544_drawBitmap(uint8_t x, uint8_t y, const uint8_t bitmap[],uint8_t w, uint8_t h) {
	  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	  uint8_t b = 0;

	  for (int16_t j = 0; j < h; j++, y++) {
	    for (int16_t i = 0; i < w; i++) {
	      if (i & 7)
	        b <<= 1;
	      else
	        b = bitmap[j * byteWidth + i / 8];
	      if (b & 0x80)
	        PCD8544_drawPixel(x + i, y, 1);
	    }
	  }
}

void PCD8544_drawRoundRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t r) {
  int8_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
  if (r > max_radius)
    r = max_radius;
  // smarter version
  PCD8544_drawHorizontalLine(x + r, y, w - 2 * r);         // Top
  PCD8544_drawHorizontalLine(x + r, y + h - 1, w - 2 * r); // Bottom
  PCD8544_drawVerticalLine(x, y + r, h - 2 * r);         // Left
  PCD8544_drawVerticalLine(x + w - 1, y + r, h - 2 * r); // Right
  // draw four corners
  PCD8544_drawCircleHelper(x + r, y + r, r, 1);
  PCD8544_drawCircleHelper(x + w - r - 1, y + r, r, 2);
  PCD8544_drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4);
  PCD8544_drawCircleHelper(x + r, y + h - r - 1, r, 8);
}

void PCD8544_drawFilledRoundRectangle(int8_t x, int8_t y, int8_t w, int8_t h, int8_t r) {
  int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
  if (r > max_radius)
    r = max_radius;
  // smarter version
  PCD8544_drawFilledRectangle(x + r, y, w - 2 * r, h);
  // draw four corners
  PCD8544_drawFilledCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1);
  PCD8544_drawFilledCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1);
}

void PCD8544_drawTriangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
  PCD8544_drawLine(x0, y0, x1, y1);
  PCD8544_drawLine(x1, y1, x2, y2);
  PCD8544_drawLine(x2, y2, x0, y0);
}

void PCD8544_printTime(uint8_t hour, uint8_t minute){
	 PCD8544_drawBitmap(0,15,seven_segment_font[hour / 10],16,28);
	 PCD8544_drawBitmap(20,15,seven_segment_font[hour % 10],16,28);

	 PCD8544_drawBitmap(48,15,seven_segment_font[minute / 10],16,28);
	 PCD8544_drawBitmap(68,15,seven_segment_font[minute % 10],16,28);
}

void PCD8544_drawClockDeliminator(){
	 PCD8544_drawFilledRectangle(40,23,4,4);
	 PCD8544_drawFilledRectangle(40,32,4,4);
}

uint8_t PCD8544_getContrast(){
	return _contrast;
}

