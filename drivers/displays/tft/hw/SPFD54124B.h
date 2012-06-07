/**************************************************************************/
/*!
    @file     SPFD54124B.h
    @author   S. Dixon

    @section LICENSE

    Software License Agreement (BSD License)

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
#ifndef __SPFD54124B_H__
#define __SPFD54124B_H__

#include "projectconfig.h"
#include "drivers/displays/tft/lcd.h"

/**************************************************************************
    SPFD54124B CONNECTOR
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   LED+
      2   VDD             VCC
      3   GND
      4   GND
      5   SCLK
      6   GND
      7   GND
      8   GND
      9   GND
     10   GND
     11                   N/C
     12   LED_RST
     13   LED_CS
     14   GND
     15   GND
     16   GND
     17   GND
     18   MOSI            SDA
     19   GND
     20   GND
     21   VCC
     22   GND

 **************************************************************************/

// Control pins
#define SPFD54124B_GPIODATAREG     GPIO_GPIO0DATA
#define SPFD54124B_GPIODATAREG1    GPIO_GPIO1DATA
// Port 0 pins
#define SPFD54124B_PORT            (0)
#define SPFD54124B_SCL_PIN         (6)
#define SPFD54124B_SDA_PIN         (9)
#define SPFD54124B_BL_PIN          (11)
// Port 1 pins
#define SPFD54124B_PORT1           (1)
#define SPFD54124B_CS_PIN          (8)
#define SPFD54124B_RES_PIN         (4)

// Macros for control line state
#define CLR_SDA     do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_SDA_PIN); } while(0)
#define SET_SDA     do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_SDA_PIN); SPFD54124B_GPIODATAREG |= (1<<SPFD54124B_SDA_PIN); } while(0)
#define CLR_SCL      do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_SCL_PIN); } while(0)
#define SET_SCL      do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_SCL_PIN); SPFD54124B_GPIODATAREG |= (1<<SPFD54124B_SCL_PIN); } while(0)
#define CLR_CS       do { SPFD54124B_GPIODATAREG1 &= ~(1<<SPFD54124B_CS_PIN); } while(0)
#define SET_CS       do { SPFD54124B_GPIODATAREG1 &= ~(1<<SPFD54124B_CS_PIN); SPFD54124B_GPIODATAREG1 |= (1<<SPFD54124B_CS_PIN); } while(0)
#define CLR_RES      do { SPFD54124B_GPIODATAREG1 &= ~(1<<SPFD54124B_RES_PIN); } while(0)
#define SET_RES      do { SPFD54124B_GPIODATAREG1 &= ~(1<<SPFD54124B_RES_PIN); SPFD54124B_GPIODATAREG1 |= (1<<SPFD54124B_RES_PIN); } while(0)
#define CLR_BL       do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_BL_PIN); } while(0)
#define SET_BL       do { SPFD54124B_GPIODATAREG &= ~(1<<SPFD54124B_BL_PIN); SPFD54124B_GPIODATAREG |= (1<<SPFD54124B_BL_PIN); } while(0)

//#define SPFD54124B_NOP      (0x0)
#define SPFD54124B_NOP      (0x25)
#define SPFD54124B_SWRESET  (0x01)
#define SPFD54124B_SLPIN    (0x10)
#define SPFD54124B_SLPOUT   (0x11)
#define SPFD54124B_PTLON    (0x12)
#define SPFD54124B_NORON    (0x13)
#define SPFD54124B_INVOFF   (0x20)
#define SPFD54124B_INVON    (0x21)
#define SPFD54124B_DISPON   (0x29)
#define SPFD54124B_CASET    (0x2A)
#define SPFD54124B_RASET    (0x2B)
#define SPFD54124B_RAMWR    (0x2C)
#define SPFD54124B_RGBSET   (0x2D)
#define SPFD54124B_COLMOD   (0x3A)
#define SPFD54124B_MADCTL   (0x36)
#define SPFD54124B_VSCSAD   (0x37)
#define SPFD54124B_FRMCTR1  (0xB1)
#define SPFD54124B_INVCTR   (0xB4)
#define SPFD54124B_DISSET5  (0xB6)
#define SPFD54124B_PWCTR1   (0xC0)
#define SPFD54124B_PWCTR2   (0xC1)
#define SPFD54124B_PWCTR3   (0xC2)
#define SPFD54124B_VMCTR1   (0xC5)
#define SPFD54124B_PWCTR6   (0xFC)
#define SPFD54124B_GMCTRP1  (0xE0)
#define SPFD54124B_GMCTRN1  (0xE1)

#endif
