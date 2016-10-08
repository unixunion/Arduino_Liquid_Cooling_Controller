#include "pgfx.h"
#include "Arduino.h"

// pgfx_HBAR::pgfx_HBAR(int x, int y, int w, int h, char * name, U8GLIB_SSD1306_128X64 & display)
// {
//   _x = x;
//   _y = y;
//   _width = w;
//   _height = h;
//   _display = & display;
//   _name = name;
// }


/*

  HBAR

*/

void pgfx_HBAR::update(float value, float max)
{

  if (value >= max) {
    _display->setFont(u8g_font_ncenB14);
    _display->setFontPosTop();
    _display->firstPage();
    do {
      _display->drawStr(16, 0, F("WARNING"));
      _display->drawStr(48, 24, _name);
    } while( _display->nextPage() );
    tone(_spkr_pin, 80, 125);
    delay(250);
  } else {
  
    _display->setFont(u8g_font_profont12);
    _display->setFontPosTop();
    _display->drawStr(_x, _y, _name);
    _display->drawRFrame(_x, _y+16, _width, _height-32, 4);
    if (max >0 && value > 1) {
      unsigned int fp = (value / max) * (_height-36);
      _display->drawBox(_x+2, _y+18 + ((_height-36) - fp), _width-4, fp);
    }
    char str_buffer[4];
    dtostrf(value, 1, 0, str_buffer);
    _display->setFont(u8g_font_ncenB14);
    _display->setFontPosTop();
    _display->drawStr(_x, _y+20 + (_height-36), str_buffer );
  }

}

void pgfx_HBAR::update(float v1, float m1, float v2, float m2)
{
  _display->drawStr(_x, _y, _name);

  _display->drawRFrame(_x, _y+16, _width, _height-32, 4);
  if (m1 > 0 && v1 > 1) {
    unsigned int fp1 = (v1 / m1) * (_height-36);
    _display->drawBox(_x+2, _y+18 + ((_height-36) - fp1), (_width/2)-3, fp1);
  }
  if (m2 > 0 && v2 > 1) {
    unsigned int fp2 = (v2 / m2) * (_height-36);
    _display->drawBox(_x+(_width/2)+1, _y+18 + ((_height-36) - fp2), (_width/2)-3, fp2);
  }
  
}

/*

  BUTTON

*/


