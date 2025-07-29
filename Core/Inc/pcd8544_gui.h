#ifndef INC_PCD8544_GUI_H_
#define INC_PCD8544_GUI_H_

#include "pcd8544_gui.h"
#include "pcd8544_driver.h"

void PCD8544_GUI_drawProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress);
void PCD8544_GUI_displayMenuName(char* name);
void PCD8544_GUI_drawSideBar(uint8_t x, uint8_t y, uint8_t length, uint8_t index, uint8_t totalIndexes);

#endif /* INC_PCD8544_GUI_H_ */
