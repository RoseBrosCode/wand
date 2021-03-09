/*
  Capacitive Touch Surface Client

  Manages a Touch Surface Input
*/

#ifndef TOUCH_CLIENT
#define TOUCH_CLIENT

#include "Arduino.h"

// Definitions of touch events
#define NO_INTERACTION 0
#define SINGLE_TAP 1
#define DOUBLE_TAP 2
#define HOLDING 3

// Establishes the pin that the touch surface is attached to
void setupTouchSurface(int touchPin);

// Returns an int representing the UI action a user has taken with the touch surface 
int getTouchEvent();

#endif
