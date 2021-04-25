/*
  LED Client

  Manages a common cathode RGB LED.
*/

#ifndef LED_CLIENT
#define LED_CLIENT

#include "Arduino.h"

// Initializes RGB pins.
void setupRGBLED(int redPin, int greenPin, int bluePin);

// Sets the LED to an RGB color.
void setRGBLED(int r, int g, int b);

// Sets the LED to the next RGB color in the colorloop.
void incrementRGBColorloop();

#endif
