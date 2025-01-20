/*
 * UI.c
 *
 *  Created on: Jan 2, 2025
 *      Author: Admin
 */

#include "ILI9341_STM32_Driver.h"
#include "stdbool.h"
#include "fonts.h"

void show_menu(){

	// initialise the lcd
	ILI9341_Init();
	// setting rotation of the lcd
	ILI9341_SetRotation(SCREEN_HORIZONTAL_2);

	ILI9341_FillScreen(WHITE);

	static char BufferText[30];

	sprintf(BufferText, "1.SHOW RTC TIME");
	ILI9341_DrawText(BufferText, FONT4, 10, 10, BLACK, WHITE);

	sprintf(BufferText, "2.SET RTC TIME");
	ILI9341_DrawText(BufferText, FONT4, 10, 30, BLACK, WHITE);





}


