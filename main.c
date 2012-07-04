/**************************************************************************/
/*! 
 @file     main.c
 @author   K. Townsend (microBuilder.eu)

 @section LICENSE

 Software License Agreement (BSD License)

 Copyright (c) 2012, microBuilder SARL
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "projectconfig.h"
#include "sysinit.h"

#include "core/gpio/gpio.h"
#include "core/systick/systick.h"

#include "drivers/displays/tft/fonts/dejavusans9.h"
#include "drivers/displays/tft/fonts/dejavusansbold9.h"
#include "drivers/displays/tft/fonts/dejavusansmono8.h"
#include "drivers/displays/tft/fonts/dejavusanscondensed9.h"

#include "drivers/displays/tft/lcd.h"    
#include "drivers/displays/tft/drawing.h"  
#include "drivers/displays/tft/touchscreen.h"
#include "drivers/displays/tft/controls/button.h"
#include "drivers/displays/tft/bmp.h"
#include "drivers/displays/tft/theme.h"

void marquee(uint16_t width, uint16_t height, uint16_t color,
		uint16_t background) {
	int lineWidth = 5;
	drawRectangleFilled(0, 0, width, lineWidth, color);
	drawRectangleFilled(width - lineWidth, 0, width, height, color);
	drawRectangleFilled(0, height - lineWidth, width, height, color);
	drawRectangleFilled(0, 0, lineWidth, height, color);
}
/**************************************************************************/
/*! 
 Main program entry point.  After reset, normal code execution will
 begin here.
 */
/**************************************************************************/
int main(void) {
	int i, j, k;
	int h, w;
	unsigned char first = 1;
	char filename[] = "/image0.bmp";
	char filec = '0';
	// Configure cpu and mandatory peripherals
	systemInit();
#ifdef CFG_BRD_LPC1343_ARMBY
	// Disable the internal pullup/down resistor on PB
	gpioSetPullup(&CFG_PB_IOCON, gpioPullupMode_Inactive);
	systickDelay(30);
	// Setup an interrupt on Power button
	gpioSetInterrupt(CFG_PB_PORT, // Port
			CFG_PB_PIN, // Pin
			gpioInterruptSense_Edge, // Edge Sensitive
			gpioInterruptEdge_Single, // Single Edge
			gpioInterruptEvent_ActiveHigh); // Active High
	gpioIntClear(CFG_PB_PORT, CFG_PB_PIN);
	// Enable the interrupt
	gpioIntEnable(CFG_PB_PORT, CFG_PB_PIN);
    //  Setup GPIO function on MB (middle button) pin
	//  This may mess up USB functionality based on ROM USB code?
//    CFG_MB_IOCON &= ~CFG_MB_MASK;
//    CFG_MB_IOCON |= CFG_MB_FUNC_GPIO;
//    gpioSetPullup(&CFG_MB_IOCON, gpioPullupMode_PullUp);
#endif
#if !defined CFG_TFTLCD
#error "CFG_TFTLCD must be enabled in projectconfig.h for this test"
#endif
#if defined CFG_INTERFACE
#error "CFG_INTERFACE must be disabled in projectconfig.h for this test (to save space)"
#endif
//  lcdInit();
	printf("Hello, world\n");

  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;
  
	while (1) {
		// Clear the screen
		// ---------------------------------------------------------------------
//  drawFill(COLOR_WHITE);
//  drawRectangle(10,10,120,120,COLOR_BLACK);
//  for (j=0;j<10000;j++) {
//	  for (i=11;i<119;i++) {
//		  if (!first) {
//			  drawLine(i,60+sin((i+j-1-11.0)/5.0)*20,i+1,60+sin((i+j-1-10.0)/5.0)*20,COLOR_WHITE);
//		  }
//		  drawLine(i,60+sin((i+j-11.0)/5.0)*20,i+1,60+sin((i+j-10.0)/5.0)*20,COLOR_RED);
//	  }
//	  //    systickDelay(10);
//	  first = 0;
//  }
		filename[6] = filec;
		bmp_error_t error = bmpDrawBitmap(0, 0, filename);
		if (filec == '9') {
			filec = '0';
		} else {
			filec++;
		}

		if (error) {
			switch (error) {
			case BMP_ERROR_SDINITFAIL:
				break;
			case BMP_ERROR_FILENOTFOUND:
				filec = '0';
				continue;
			case BMP_ERROR_NOTABITMAP:
				// First two bytes of image not 'BM'
				break;
			case BMP_ERROR_INVALIDBITDEPTH:
				// Image is not 24-bits
				break;
			case BMP_ERROR_COMPRESSEDDATA:
				// Image contains compressed data
				break;
			case BMP_ERROR_INVALIDDIMENSIONS:
				// Width or Height is > LCD size
				break;
			case BMP_ERROR_PREMATUREEOF:
				// EOF unexpectedly reached in pixel data
				break;
			}
		}

		h = lcdGetHeight();
		w = lcdGetWidth();

//		for (i = 0; i < 20; i++) {
//			marquee(w, h, THEME_COLOR_SKYBLUE_BASE, COLOR_BLACK);
//			systickDelay(100);
//			marquee(w, h, COLOR_BLACK, COLOR_RED);
//			systickDelay(100);
//		}
//		for (i = 0; i < (h - 16); i += 16) {
//			for (j = 0; j < (w - 8); j += 8) {
//				drawRectangleFilled(j, i, j + 8, i + 8, COLOR_BLACK);
//				systickDelay(5);
//			}
//			for (j = (w - 8); j > 0; j -= 8) {
//				drawRectangleFilled(j, i + 8, j + 8, i + 16, COLOR_BLACK);
//				systickDelay(5);
//			}
//		}
//		for (i = 0; i < 5; i++) {
//			drawFill(THEME_COLOR_SKYBLUE_BASE);
//			systickDelay(50);
//			drawFill(THEME_COLOR_SKYBLUE_DARKER);
//			systickDelay(50);
//			drawFill(THEME_COLOR_SKYBLUE_LIGHTER);
//			systickDelay(50);
//		}
		for (k=0;k<300;k++) {
			if (gpioGetValue(CFG_TB_PORT,CFG_TB_PIN) == 0) {
				drawFill(COLOR_BLUE);
//			} else if (gpioGetValue(CFG_MB_PORT,CFG_MB_PIN) == 0) {
//				drawFill(COLOR_GREEN);
			} else if (gpioGetValue(CFG_BB_PORT,CFG_BB_PIN) == 0) {
				drawFill(COLOR_RED);
				printf("Pressed Red\n\r");
			}
			systickDelay(10);
		}
	}
	return 0;
}
