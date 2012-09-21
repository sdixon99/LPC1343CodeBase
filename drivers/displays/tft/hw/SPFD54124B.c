/**************************************************************************/
/*!
    @file     SPFD54124B.c
    @author   Scott
    Original (st7735) by K. Townsend (microBuilder.eu)

    @section  DESCRIPTION

    Driver for SPFD54124B 132x162 pixel TFT LCD displays. (Nokia XXXX)
    Modified from the st7735 code.  Uses 9 bit mode on the SPI interface
    so the mode is switched from 8 bit to 9 bit on the fly.

    This driver uses a 16-bit RGB565 colour palette.

    @section  LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2010, microBuilder SARL
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
#include "SPFD54124B.h"
#include "core/systick/systick.h"
#include "core/gpio/gpio.h"
#include "core/timer32/timer32.h"
#include "core/ssp/ssp.h"

static lcdOrientation_t lcdOrientation = LCD_ORIENTATION_PORTRAIT;
static lcdProperties_t SPFD54124BProperties = { 132, 162, FALSE, TRUE, FALSE };
static int spiData[8];

/*************************************************/
/* Private Methods                               */
/*************************************************/

/*************************************************/
void SPFD54124BWriteCmd(uint8_t command)
{
	setSPIwidth9();
//  CLR_CS;
////  CLR_RS;
//  CLR_SDA;
//  CLR_SCL;
//  SET_SCL;
//  uint8_t i = 0;
//  for (i=0; i<8; i++)
//  {
//    if (command & 0x80)
//    {
//      SET_SDA;
//    }
//    else
//    {
//      CLR_SDA;
//    }
//    CLR_SCL;
//    command <<= 1;
//    SET_SCL;
//  }
//  SET_CS;
	spiData[0] = (int)command;
  CLR_CS;
//	ssp0Select();
	sspSend(0, (int *)&spiData, 1);
  SET_CS;
	setSPIwidth8();
//	ssp0Deselect();
}

/*************************************************/
void SPFD54124BWriteData(uint8_t data)
{
	setSPIwidth9();
//  CLR_CS;
////  SET_RS;
//  SET_SDA;
//  CLR_SCL;
//  SET_SCL;
//  uint8_t i = 0;
//  for (i=0; i<8; i++)
//  {
//    if (data & 0x80)
//    {
//      SET_SDA;
//    }
//    else
//    {
//      CLR_SDA;
//    }
//    CLR_SCL;
//    data <<= 1;
//    SET_SCL;
//  }
//  SET_CS;
	spiData[0] = (int)(data | 0x100);
  CLR_CS;
//	ssp0Select();
	sspSend(0, (int *)&spiData, 1);
  SET_CS;
	setSPIwidth8();
//	ssp0Deselect();
}

inline void SPFD54124BWriteData2(uint8_t data1, uint8_t data2) {
	spiData[0] = (int)(data1 | 0x100);
        spiData[1] = (int)(data2 | 0x100);
  CLR_CS;
	sspSend(0, (int *)&spiData, 2);
  SET_CS;


}

/*************************************************/
void SPFD54124BSetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  SPFD54124BWriteCmd(SPFD54124B_CASET);   // column addr set
  SPFD54124BWriteData(0x00);
  SPFD54124BWriteData(x0);          // XSTART
  SPFD54124BWriteData(0x00);
  SPFD54124BWriteData(x1);          // XEND

  SPFD54124BWriteCmd(SPFD54124B_RASET);   // row addr set
  SPFD54124BWriteData(0x00);
  SPFD54124BWriteData(y0);          // YSTART
  SPFD54124BWriteData(0x00);
  SPFD54124BWriteData(y1);          // YEND
}

/*************************************************/
void SPFD54124BInitDisplay(void)
{
  SPFD54124BWriteCmd(SPFD54124B_SWRESET); // software reset
  systickDelay(100);
  CLR_CS;;
  SPFD54124BWriteCmd(SPFD54124B_SLPOUT);  // out of sleep mode
  systickDelay(200);
  SET_CS;

  CLR_RES;
  systickDelay(5);
  SET_RES;
  systickDelay(100);
  CLR_SCL;
  CLR_CS;
  SPFD54124BWriteCmd(0xBA);
  SPFD54124BWriteData(0x07);
  SPFD54124BWriteData(0x15);
  SPFD54124BWriteCmd(0x25);
  SPFD54124BWriteData(0x3F);
  SPFD54124BWriteCmd(SPFD54124B_SLPOUT);
  SPFD54124BWriteCmd(SPFD54124B_NORON);
  SPFD54124BWriteCmd(SPFD54124B_VSCSAD);
  SPFD54124BWriteData(0x00);
  SPFD54124BWriteCmd(SPFD54124B_COLMOD);
  SPFD54124BWriteData(0x05);
  SPFD54124BWriteCmd(SPFD54124B_DISPON);
  SPFD54124BWriteCmd(SPFD54124B_INVOFF);
  SPFD54124BWriteCmd(SPFD54124B_NORON);
  SPFD54124BWriteCmd(SPFD54124B_RGBSET);
  int i;
  for (i = 0; i < 32; i++)
	  SPFD54124BWriteData(i<<1);
  for (i = 0; i < 64; i++)
	  SPFD54124BWriteData(i);
  for (i = 0; i < 32; i++)
	  SPFD54124BWriteData(i<<1);

  CLR_CS;
//  systickDelay(500);
}

/*************************************************/
/* Public Methods                                */
/*************************************************/

/*************************************************/
void lcdInit(void)
{
  SET_CS;
  SET_RES;
  SET_SDA;
  CLR_SCL;
  // Set control pins to output
//  gpioSetDir(SPFD54124B_PORT, SPFD54124B_RS_PIN, 1);
  gpioSetDir(SPFD54124B_PORT, SPFD54124B_SDA_PIN, 1);
  gpioSetDir(SPFD54124B_PORT, SPFD54124B_SCL_PIN, 1);
  gpioSetDir(SPFD54124B_PORT, SPFD54124B_BL_PIN, 1);
  gpioSetDir(SPFD54124B_PORT1, SPFD54124B_CS_PIN, 1);
  gpioSetDir(SPFD54124B_PORT1, SPFD54124B_RES_PIN, 1);
  systickDelay(120);
  sspInit(0, sspClockPolarity_High, sspClockPhase_FallingEdge);

  CLR_CS;

  // Reset display
//  CLR_RES;
//  systickDelay(50);
//  SET_RES;

  // Run LCD init sequence
  SPFD54124BInitDisplay();

  // Fill black
  lcdFillRGB(COLOR_BLACK);

  // Turn backlight on
  lcdBacklight(TRUE);
}

/*************************************************/
void lcdBacklight(bool state)
{
	// Set the backlight
	if (state) {
		/* Enable the clock for CT32B0 */
		SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_CT32B0);

		/* Configure PIO0.11 as Timer0_32 MAT3 Output */
		IOCON_JTAG_TDI_PIO0_11 &= ~IOCON_JTAG_TDI_PIO0_11_FUNC_MASK;
		IOCON_JTAG_TDI_PIO0_11 |= IOCON_JTAG_TDI_PIO0_11_FUNC_CT32B0_MAT3;
		gpioSetDir(SPFD54124B_PORT, SPFD54124B_BL_PIN, 1);

		/* Set period (MR3) to 500KHz */
		TMR_TMR32B0MR0 = 2*TIMER32_CCLK_1US;

		/* Set Duty Cycle to 50% */
		TMR_TMR32B0MR3 = 1*TIMER32_CCLK_1US;

		/* Configure match control register to reset on MR0 */
		TMR_TMR32B0MCR = (TMR_TMR32B0MCR_MR0_RESET_ENABLED);

		/* External Match Register Settings for PWM */
		TMR_TMR32B0EMR = TMR_TMR32B0EMR_EMC0_TOGGLE | TMR_TMR32B0EMR_EM0;

		/* Enable PWM0 and PWM3 */
		//	    TMR_TMR32B0PWMC = TMR_TMR32B0PWMC_PWM0_ENABLED  | TMR_TMR32B0PWMC_PWM3_ENABLED;
		TMR_TMR32B0PWMC = TMR_TMR32B0PWMC_PWM3_ENABLED;

		/* Enable Timer0 */
		TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;
	} else
		TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_DISABLED;
	//    SET_BL;
}

/*************************************************/
void lcdTest(void)
{
  uint8_t i = 0;
  for (i = 0; i < 100; i++)
  {
    lcdDrawPixel(i, i, 0xFFFF);
  }
}

/*************************************************/
void lcdFillRGB(uint16_t color)
{
  uint8_t x, y;
  SPFD54124BSetAddrWindow(0, 0, lcdGetWidth() - 1, lcdGetHeight() - 1);
  SPFD54124BWriteCmd(SPFD54124B_RAMWR);  // write to RAM
  setSPIwidth9();
  for (x=0; x < lcdGetWidth(); x++)
  {
    for (y=0; y < lcdGetHeight(); y++)
    {
//      SPFD54124BWriteData(color >> 8);
//      SPFD54124BWriteData(color);
    	SPFD54124BWriteData2(color >> 8, color);
    }
  }
  SPFD54124BWriteCmd(SPFD54124B_NOP);
}

/*************************************************/
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  SPFD54124BSetAddrWindow(x,y,x+1,y+1);
  SPFD54124BWriteCmd(SPFD54124B_RAMWR);  // write to RAM
  setSPIwidth9();
//  SPFD54124BWriteData(color >> 8);
//  SPFD54124BWriteData(color);
  SPFD54124BWriteData2(color >> 8, color);
  setSPIwidth8();
}

/*************************************************/
void lcdDrawHLine(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
  // Allows for slightly better performance than setting individual pixels
  uint16_t x, pixels;

  if (x1 < x0)
  {
    // Switch x1 and x0
    x = x1;
    x1 = x0;
    x0 = x;
  }

  // Check limits
  if (x1 >= lcdGetWidth())
  {
    x1 = lcdGetWidth() - 1;
  }
  if (x0 >= lcdGetWidth())
  {
    x0 = lcdGetWidth() - 1;
  }

  SPFD54124BSetAddrWindow(x0, y, lcdGetWidth(), y + 1);
  SPFD54124BWriteCmd(SPFD54124B_RAMWR);  // write to RAM
  setSPIwidth9();
  for (pixels = 0; pixels < x1 - x0 + 1; pixels++)
  {
//    SPFD54124BWriteData(color >> 8);
//    SPFD54124BWriteData(color);
   	SPFD54124BWriteData2(color >> 8, color);
  }
  SPFD54124BWriteCmd(SPFD54124B_NOP);
  setSPIwidth8();
}

/*************************************************/
void lcdDrawVLine(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
  // Allows for slightly better performance than setting individual pixels
  uint16_t y, pixels;

  if (y1 < y0)
  {
    // Switch y1 and y0
    y = y1;
    y1 = y0;
    y0 = y;
  }

  // Check limits
  if (y1 >= lcdGetHeight())
  {
    y1 = lcdGetHeight() - 1;
  }
  if (y0 >= lcdGetHeight())
  {
    y0 = lcdGetHeight() - 1;
  }

  SPFD54124BSetAddrWindow(x, y0, x, lcdGetHeight());
  SPFD54124BWriteCmd(SPFD54124B_RAMWR);  // write to RAM
  setSPIwidth9();
  for (pixels = 0; pixels < y1 - y0 + 1; pixels++)
  {
//    SPFD54124BWriteData(color >> 8);
//    SPFD54124BWriteData(color);
   	SPFD54124BWriteData2(color >> 8, color);
  }
  setSPIwidth8();
  SPFD54124BWriteCmd(SPFD54124B_NOP);
}

/*************************************************/
uint16_t lcdGetPixel(uint16_t x, uint16_t y)
{
  // ToDo
  return 0;
}

/*************************************************/
void lcdSetOrientation(lcdOrientation_t orientation)
{
	uint16_t tmp;
	if (orientation == LCD_ORIENTATION_PORTRAIT) {
		SPFD54124BWriteCmd(SPFD54124B_MADCTL);
		SPFD54124BWriteData(0x0);
		tmp = SPFD54124BProperties.height;
		SPFD54124BProperties.height = SPFD54124BProperties.width;
		SPFD54124BProperties.width = tmp;
	} else {
		SPFD54124BWriteCmd(SPFD54124B_MADCTL);
		SPFD54124BWriteData(0xA0);
		tmp = SPFD54124BProperties.height;
		SPFD54124BProperties.height = SPFD54124BProperties.width;
		SPFD54124BProperties.width = tmp;
	}
	lcdOrientation = orientation;
}

/*************************************************/
lcdOrientation_t lcdGetOrientation(void)
{
  return lcdOrientation;
}

/*************************************************/
uint16_t lcdGetWidth(void)
{
  return SPFD54124BProperties.width;
}

/*************************************************/
uint16_t lcdGetHeight(void)
{
  return SPFD54124BProperties.height;
}

/*************************************************/
void lcdScroll(int16_t pixels, uint16_t fillColor)
{
  // ToDo
}

/*************************************************/
uint16_t lcdGetControllerID(void)
{
  // ToDo
  return 0;
}

/*************************************************/
lcdProperties_t lcdGetProperties(void)
{
  return SPFD54124BProperties;
}
