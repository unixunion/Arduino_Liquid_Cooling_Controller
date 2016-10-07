#include "pgfx.h"

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

  _display->drawStr(_x, _y, _name);
  _display->drawRFrame(_x, _y+16, _width, _height-16, 4);
  if (max >0 && value > 1) {
    unsigned int fp = (value / max) * (_height-20);
    _display->drawBox(_x+2, _y+18 + ((_height-20) - fp), _width-4, fp);
  }

}

void pgfx_HBAR::update(float v1, float m1, float v2, float m2)
{
  _display->drawStr(_x, _y, _name);

  _display->drawRFrame(_x, _y+16, _width, _height-16, 4);
  if (m1 > 0 && v1 > 1) {
    unsigned int fp1 = (v1 / m1) * (_height-20);
    _display->drawBox(_x+2, _y+18 + ((_height-20) - fp1), (_width/2)-3, fp1);
  }
  if (m2 > 0 && v2 > 1) {
    unsigned int fp2 = (v2 / m2) * (_height-20);
    _display->drawBox(_x+(_width/2)+1, _y+18 + ((_height-20) - fp2), (_width/2)-3, fp2);
  }
  
}

/*

  BUTTON

*/


