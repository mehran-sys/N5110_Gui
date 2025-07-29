#include "pcd8544_gui.h"

void PCD8544_GUI_drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress){
	PCD8544_drawRectangle(x, y, x + width, y + height);
	PCD8544_drawFilledRectangle(x+2, y+2, (uint8_t)((progress)*(width-3)/100), height-4);
}

void PCD8544_GUI_displayMenuName(char* name){
	PCD8544_drawHorizontalLine(0, 12, PCD8544_WIDTH);
	PCD8544_printStringAlign(name, 2, ALIGNMETN_CENTER);
}

void PCD8544_GUI_drawSideBar(uint8_t x, uint8_t y, uint8_t length, uint8_t index, uint8_t totalIndexes){
	PCD8544_drawVerticalLine(x, y, length);
	PCD8544_drawFilledRectangle(x-1, (y+2) + (index * (length / totalIndexes)), 3, 4);
}
