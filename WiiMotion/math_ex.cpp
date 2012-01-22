#include "math_ex.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

float arctan2(float y, float x) 
{
  // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
  float coeff_1 = PI/4;
  float coeff_2 = 3*coeff_1;
  float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
  float r, angle;
  
  if (x >= 0) {
    r = (x - abs_y) / (x + abs_y);
    angle = coeff_1 - coeff_1 * r;
  }
  else {
    r = (x + abs_y) / (abs_y - x);
    angle = coeff_2 - coeff_1 * r;
  }

  if (y < 0)
    return(-angle);     // negate if in quad III or IV
  else
    return(angle);
} 
