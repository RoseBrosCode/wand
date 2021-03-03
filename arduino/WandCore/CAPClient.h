/*
  Capacitive Touch Client

  Manages a Touch Input
*/

#ifndef CAP_CLIENT
#define CAP_CLIENT

#include "Arduino.h"

/* checkTouchSurface
 * Takes a pin attached to a touch surface
 * Returns an int representing the UI action a user has taken with the touch surface 
 * 
 * 0 = not interacting with the surface
 * 1 = user single tapped
 * 2 = user double tapped
 * 3 = user tapped and is holding
 */
int getTouchEvent(int pin);

#endif
