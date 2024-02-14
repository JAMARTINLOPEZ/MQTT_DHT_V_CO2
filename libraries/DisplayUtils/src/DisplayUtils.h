/**
 * Defines class DisplayArea for working with areas on displays (shocking, eh?)
 * and DisplayNumber class for fast flicker-free drawing of numbers.
 *
 * GPLv2 Licence https://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 * 
 * Copyright (c) 2022 Philip Fletcher <philip.fletcher@stutchbury.com>
 * 
 */


#ifndef DisplayUtils_h
#define DisplayUtils_h

#include "Arduino.h"

/**
 * A struct that holds x, y coordinates (often used as a return type).
 * Have to use int or throws "warning: narrowing conversion of..."
 * at every opportunity eg with the slightest touch of simple
 * arithmetic operations such as round() or '+ 1' etc...
 * Go figure...
 */
struct Coords_s {
  int  x;
  int  y;
};

/**
 * A class that defines an area on the display and provides a host
 * of utility methods for determining the absolute display x, y
 * posion of various attributes such as b() (bottom), l() (left),
 * cp() (centre point) etc.
 * Method names are deliberately kept short (concise, or terse, even)
 * because they are frequently called withing the arguments of other
 * class methods.
 */
class DisplayArea {
protected:
  uint16_t _x = 0;
  uint16_t _y = 0;
  uint16_t _w = 0;
  uint16_t _h = 0;
public:
  DisplayArea() {} //Used only by sub classes
  /**
   * Construct a new area
   */
  DisplayArea(uint16_t  x, uint16_t  y, uint16_t  w, uint16_t  h)
    : _x(x), _y(y), _w(w), _h(h) { }

  /**
   * Returns the absolute x position
   */
  uint16_t  x() {
    return _x;
  }
  /**
   * Returns the absolute y position
   */
  uint16_t  y() {
    return _y;
  }
  /**
   * Returns the width
   */
  virtual uint16_t  w() {
    return _w;
  }
  /**
   * Returns the height
   */
  virtual uint16_t  h() {
    return _h;
  }
  /**
   * Set a new x position
   */
  void setX(uint16_t  x) {
    _x = x;
  }
  /**
   * Set a new y position
   */
  void setY(uint16_t  y) {
    _y = y;
  }
  /**
   * Set a new width
   */
  void setW(uint16_t  w) {
    _w = w;
  }
  /**
   * Set a new height
   */
  void setH(uint16_t  h) {
    _h = h;
  }
  /**
   * Returns right x absolute position
   */
  uint16_t  r() {
    return _x + w()-1; //GFX draws rectangles one line in
  }
  /**
   * Returns bottom y absolute position
   */
  uint16_t  b() {
    return _y + h()-1;  //GFX draws rectangles one line up
  }
  /**
   * Returns height divided by div (default 2)
   */
  uint16_t  hDiv(uint8_t div = 2) {
    return floor(h() / (div == 0 ? 1 : div));
  }
  /**
   * Returns nth absolute y position of height divided by div
   */
  uint16_t  yDiv(uint8_t div = 2, uint8_t nth = 0) {
    return (_y + (nth * hDiv(div)));
  }
  /**
   * Returns nth absolute Y position of bottom with height divided by div
   */
  uint16_t  bDiv(uint8_t div = 2, uint8_t nth = 0) {
    return yDiv(div, nth) + (hDiv(div));
  }
  /**
   * Returns absolute Y position of centre line with height divided by div
   */
  uint16_t  yCl(uint8_t div = 1, uint8_t nth = 0) {
    return yDiv(div, nth) + floor(hDiv(div) / 2);
  }
  /**
   * Returns width divided by div (default 2)
   */
  uint16_t  wDiv(uint8_t div = 2) {
    return w() / (div == 0 ? 1 : div);
  }
  /**
   * Returns nth absolute X position of width divided by div
   */
  uint16_t  xDiv(uint8_t div = 2, uint8_t nth = 0) {
    return _x + (nth * wDiv(div));
  }
  /**
   * Returns nth absolute right position of width divided by div
   */
  uint16_t  rDiv(uint8_t div = 2, uint8_t nth = 0) {
    return xDiv(div, nth) + (wDiv(div));
  }
  /**
   * Returns absolute X position of centre line with height divided by div
   */
  uint16_t  xCl(uint8_t div = 1, uint8_t nth = 0) {
    return xDiv(div, nth) + (wDiv(div) / 2);
  }
  /**
   * Returns absolute centre point as Coords_s
   */
  Coords_s cp() {
    return { xCl(), yCl() };
  }
  /**
   * Returns absolute top left as Coords_s
   */
  Coords_s tl() {
    return { _x, _y };
  }
  /**
   * Returns absolute top right as Coords_s
   */
  Coords_s tr() {
    return { r(), _y };
  }
  /**
   * Returns absolute bottom right as Coords_s
   */
  Coords_s br() {
    return { r(), b() };
  }
  /**
   * Returns absolute bottom left as Coords_s
   */
  Coords_s bl() {
    return { _x, b() };
  }
  /**
   * Returns true if area contains posX and posY
   */
  bool contains(uint16_t  posX, uint16_t  posY) {
    return _x <= posX && posX <= r() && _y <= posY && posY <= b();
  }
};

//Only available when Adafruit's GFX library is loaded
#ifdef _ADAFRUIT_GFX_H

/**
   A class for flicker free drawing of large numbers that change frequently.
   Only refreshes digits that have changed.
   Requires Adafruit's excellent GFX library and must use the 'alternate'
   fonts (located in the GFX Fonts folder)
   https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts
   Digits are drawn fixed width but a fixed width font is not essential (width
   is based on the widest 0-9 digit).
   You *must* call begin() in setup() to define font. Calling w() or h() will
   return zero if called before font is set.
   The font, format background and foreground colours can be changed at runtime.
*/

class DisplayNumber : public DisplayArea {

private:
  Adafruit_GFX& gfx;
  const GFXfont* font;

  float drawnNumber = NAN;
  char drawnBuffer[18];

  uint8_t numDigits = 7;
  uint8_t precision = 3;

  uint8_t pointPosition = 0;
  uint8_t digitWidth = 0;
  uint8_t halfDigitWidth = 0;
  uint8_t quarterDigitWidth = 0;
  uint8_t digitHeight = 0;
  uint8_t digitBaseline = 0;
  float maxNum = 0;

  uint16_t bgColour = 0;
  uint16_t drawnBgColour = 0;
  uint16_t textColour = 65535;
  uint16_t drawnTextColour = 65535;

  bool stale = true;
  bool doConfig = true;

  //Don't allow width & height to be set
  void setW(int  w) {}
  void setH(int  h) {}

  /**
     Loop through digits 0-9 to find widest & tallest
     setting digitHeight, digitWidth & digitBaseline
  */
  void setMaxTextBounds() {
    int16_t  x1, y1;
    uint16_t w1, h1;
    char digit[2]; //For UNO & other AVRs, getTextBounds only accepts char array
    gfx.setFont(font);
    gfx.setTextSize(1);
    digitWidth = 0;
    w1 = 0;
    for (uint8_t i = 0; i < 10; i++) {
      digit[0] = ('0' + i); //Get the text representation of i
      digit[1] = '\0'; //<-- This was a particularly nasty critter...
      gfx.getTextBounds(digit, 0, 0, &x1, &y1, &w1, &h1); //Always use x=0, y=0 or y1 will be wrong
      digitWidth = max(digitWidth, w1 + x1);
      digitHeight = max(digitHeight, h1);
      digitBaseline = max(digitBaseline, abs(y1));
    }
  }

  /**
     Configure required variables when parameters are changed.
  */
  void configure() {
    if (doConfig) {
      setMaxTextBounds();
      pointPosition = numDigits - precision + (precision == 0 ? 2 : 1);
      halfDigitWidth = precision == 0 ? 0 : ceil(digitWidth / 2.0);
      quarterDigitWidth = precision == 0 ? 0 : ceil(digitWidth / 4.0);
      maxNum = pow(10.f, (numDigits - precision));
      stale = true;
      //Serial.printf("num: %i, precision: %i, pointPosition: %i digitWidth: %i, half: %i, width: %i, height: %i, maxNum: %f\n", numDigits+2, precision, pointPosition, digitWidth, halfDigitWidth, w(), h(), maxNum);
    }
  }

public:
  /**
     Constructor - requires an Adafruit_GFX library (usually a sub-class)
  */
  DisplayNumber(Adafruit_GFX& d)
    : gfx(d) {
    configure();
  }

  /**
     Font *must* be set (here or in begin()) or nothing will be drawn - must be
     set after GFX begin()
     Cannot be set in constructor because GFX begin() will not yet have been called.
  */
  void setFont(const GFXfont* f) {
    if (font == NULL || font != f) {
      font = f;
      gfx.setFont(font);
      configure();
    }
  }

  /**
     Set the x, y coordinates - defaults to the top left corner - if
     alignRight is true then top right.
  */
  void setPosition(int  x, uint8_t y) {
    if (_x != x || _y != y) {
      _x = x;
      _y = y;
      configure();
    }
  }

  /**
     Set the size (total number of digits not including '-' or '.')
     and the precision of the displayed number.
     Default size is 7 (max is 15).
     Default precision is 3 must be less than size-1 (ie, we always
     display a leading zero).
  */
  void setFormat(uint8_t size = 3, uint8_t precision = 3) {
    size = max(min(size, 15), 2);
    precision = max(min(precision, (size - 1)), 0);
    if (this->numDigits != size || this->precision != precision) {
      this->numDigits = size;
      this->precision = precision;
      configure();
    }
  }

  /**
     begin() must be called from within setup() to define the font and
     optionally the position and format.
  */
  void begin(uint16_t  x, uint8_t y, const GFXfont* f, uint8_t size = 7, uint8_t precision = 3) {
    doConfig = false;
    setPosition(x, y);
    setFont(f);
    setFormat(size, precision);
    doConfig = true;
    configure();
  }

  /**
     begin() must be called from within setup() to define the font and
     optionally the position and format.
  */
  void begin(const GFXfont* f, uint8_t size = 7, uint8_t precision = 3) {
    begin(0, 0, f, size, precision);
  }



  /**
     Set the colour of the text.
     Yes, it's spelled wrong to match the GFX library
  */
  void setTextColor(uint16_t  colour = 65535) {
    if (textColour != colour) {
      textColour = colour;
      stale = true;
    }
  }

  /**
     Set the background colour.
  */
  void setBackgroundColour(uint16_t  colour = 0) {
    if (bgColour != colour) {
      bgColour = colour;
      stale = true;
    }
  }



  /**
     Overrides DisplayArea::w()
     Return the calculated width of the display area
     Note: this will return zero if the font has not been set.
  */
  uint16_t  w() override {
    return (digitWidth * (numDigits + 1)) + halfDigitWidth;
  }

  /**
     Overrides DisplayArea::h()
     Return the calculated height of the display area.
     Note: this will return zero if the font has not been set.
  */
  uint16_t  h() override {
    return (digitHeight);
  }


  /**
     Core draw method. x, y, textColour and bgColour override set values but do not change them.
     Normally call one of the override methods below.
     Returns false if font not set or number too large
  */
  bool draw(uint16_t  x, uint16_t  y, float num, uint16_t  textColour, uint16_t  bgColour, bool forceRefresh) {
    if (font == NULL) return false;
    if (!stale && !forceRefresh && num == drawnNumber && drawnTextColour == textColour && drawnBgColour == bgColour) return true;
    bool ret = true;
    //gfx.drawLine(x-1, y, x-1, y+digitHeight, ILI9341_WHITE);
    gfx.setFont(font);
    gfx.setTextSize(1);
    gfx.setTextColor(textColour);
    char newBuffer[18];
    dtostrf(num, (numDigits + 2), precision, newBuffer);
    if (num > maxNum) {
      gfx.fillRect(x, y, w(), h(), bgColour);
      gfx.setCursor(x, y + digitBaseline);
      gfx.print("Err!");
      ret = false;
    }
    else {
      for (int i = 0; i < (numDigits + 2); i++) {
        if (forceRefresh || newBuffer[i] != drawnBuffer[i]) {
          int  xPos = x + (i * digitWidth - (i > pointPosition ? halfDigitWidth : 0)) - (precision == 0 ? digitWidth : 0);
          gfx.fillRect(xPos, y, (i == pointPosition ? halfDigitWidth : digitWidth), digitHeight, bgColour);
          gfx.setCursor(xPos - (i == pointPosition ? quarterDigitWidth : 0), y + digitBaseline);
          gfx.print(newBuffer[i]);
          drawnBuffer[i] = newBuffer[i];
        }
      }
      ret = true;
    }
    drawnNumber = num;
    drawnTextColour = textColour;
    drawnBgColour = bgColour;
    stale = false;
    gfx.setFont();
    return ret;
  }

  /**
     Draw the number using the current position but override the
     colour for this draw only.
     Returns false if font not set or number too large
  */
  bool draw(float num, uint16_t  textColour, bool forceRefresh) {
    return draw(_x, _y, num, textColour, bgColour, forceRefresh);
  }

  /**
     Draw the number using the current position and colours.
     Returns false if font not set or number too large
  */
  bool draw(float num, bool forceRefresh = false) {
    return draw(_x, _y, num, textColour, bgColour, forceRefresh);
  }
};

#endif //GFX

#endif //DisplayUtils_h
