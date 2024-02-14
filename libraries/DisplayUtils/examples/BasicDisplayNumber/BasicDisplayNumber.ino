/*

Arduino example showing the basic usage of the DisplayNumber class.


MIT License

Copyright (c) 2022 Philip Fletcher <philip.fletcher@stutchbury.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice must be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Arduino.h"
#include "SPI.h"
#include "Adafruit_GFX.h"

//Use the GFX sub class appropriate for your display
#include "Adafruit_ILI9341.h"
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <DisplayUtils.h>

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

//Use the GFX sub class appropriate for your display
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

const uint16_t displayRefreshMs = 100; //Refresh the screen 10 times per second
unsigned long lastDisplayRefresh = 0;
const uint16_t counterUpdateMs = 10; //Increment the example counter 100 times per second
unsigned long lastCounterUpdate = 0;


/**
   Declare your DisplayNumber(s)
*/
DisplayNumber xAxis(tft);
DisplayNumber yAxis(tft);
DisplayNumber zAxis(tft);
DisplayNumber aAxis(tft);


float num = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);
  Serial.println("Basic DisplayNumber");
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  /**
     Call begin() *after* tft.begin() for each DisplayNumber to set the font
     (before using other methods)
  */
  xAxis.begin(&FreeMonoBold24pt7b);
  yAxis.begin(&FreeMonoBold24pt7b);
  zAxis.begin(&FreeMonoBold24pt7b);
  //Set the position, font, size and precision
  aAxis.begin(50, 150, &FreeMonoBold12pt7b, 15, 0);

  /**
     You can set the posi
  */

  //Right align and stack x, y, & z
  xAxis.setPosition(tft.width() - yAxis.w(), 0);
  yAxis.setPosition(tft.width() - yAxis.w(), xAxis.h() + 10);
  zAxis.setPosition(tft.width() - zAxis.w(), xAxis.h() + yAxis.h() + 20);
}

void loop() {
  unsigned long now = millis();
  // put your main code here, to run repeatedly:
  /**
     Note: you don't have to use this to limit the display
     refresh but it is good practice (gives your microcontroller
     a chance to do other things.
     Although DrawNumber is fast, there's a limit to how quickly the
     display can be updated - especially over SPI.
  */
  if ( now > (lastDisplayRefresh + displayRefreshMs) ) {
    lastDisplayRefresh = now;
    /**
     * Draw your numbers
     */
    xAxis.draw(num);
    //Make it a different number
    yAxis.draw(num * 13.2322355);
    /**
       Change the colours
       Warning: If passing colour you *must* pass the forceRefresh
       parameter or you will unintentionally call the wrong draw() method -
       passing your colour to the forceRefresh argument.
       This will *not* give you the results you expect!
    */
    zAxis.draw(num * -7.89, ILI9341_RED, false);
    aAxis.draw(num * 1000, ILI9341_GREEN, false);
  }
  /**
   * Do other stuff
   */
  if ( now > (lastCounterUpdate + counterUpdateMs) ) {
    lastCounterUpdate = now;
    num += 0.001;
  }
}
