/*

Arduino example showing the use of the DisplayArea class.

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

const uint16_t displayRefreshMs = 800; //Refresh the screen once per second
unsigned long lastDisplayRefresh = 0;

//Change these for your GFX sub class
const uint16_t black = ILI9341_BLACK;
const uint16_t white = ILI9341_WHITE;
const uint16_t red = ILI9341_RED;
const uint16_t green = ILI9341_GREEN;
const uint16_t blue = ILI9341_BLUE;

const uint8_t xPos = 5;
const uint8_t yPos = 5;
const uint8_t width = 150;
const uint8_t height = 50;

DisplayArea area1(xPos, yPos, width, height);

const uint8_t r1 = 5;
const uint8_t r2 = 3;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);
  Serial.println("Basic DisplayNumber");
  tft.begin();
  tft.setRotation(1);
  randomSeed(analogRead(0));
  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = millis();
  if ( now > (lastDisplayRefresh + displayRefreshMs) ) {
    lastDisplayRefresh = now;

    //Randomly position our area
    area1.setX(random(20, 150));
    area1.setY(random(15, 150));
    area1.setW(random(30, 120));
    area1.setH(random(20, 80));
    
    tft.fillScreen(black);
    //Draw a rectangle
    tft.drawRect(area1.x(), area1.y(), area1.w(), area1.h(), white);
    //Draw a circle at the centre point
    tft.fillCircle(area1.cp().x, area1.cp().y, r1, red);
    //Draw circles on the corners. Note: these methods return a Coords_s
    tft.fillCircle(area1.tl().x, area1.tl().y, r1, green);
    tft.fillCircle(area1.tr().x, area1.tr().y, r1, green);
    tft.fillCircle(area1.bl().x, area1.bl().y, r1, green);
    tft.fillCircle(area1.br().x, area1.br().y, r1, green);
    //Drwa circles on the centre of the edges
    tft.fillCircle(area1.xCl(), area1.y(), r1, blue); //top
    tft.fillCircle(area1.xCl(), area1.b(), r1, blue); //bottom
    tft.fillCircle(area1.x(), area1.yCl(), r1, blue); //left
    tft.fillCircle(area1.r(), area1.yCl(), r1, blue); //left
  }
}
