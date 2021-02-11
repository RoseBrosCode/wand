/*
  LED Client

  Manages a common cathode RGB LED.
*/

#include "LEDClient.h"

#define CHAN_0 0
#define CHAN_1 1
#define CHAN_2 2

// Initializes RGB pins.
void setupRGBLED(int redPin, int greenPin, int bluePin)
{
  // Assign PWM channels to GPIO pins.
  ledcAttachPin(redPin, 0);
  ledcAttachPin(greenPin, 1);
  ledcAttachPin(bluePin, 2);

  // Initialize PWM channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(CHAN_0, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(CHAN_1, 12000, 8);
  ledcSetup(CHAN_2, 12000, 8);

  Serial.println("Successfully set up RGB LED.");
}

// Sets the LED to an RGB color.
void setRGBLED(int r, int g, int b)
{
  ledcWrite(CHAN_0, r);
  ledcWrite(CHAN_1, g);
  ledcWrite(CHAN_2, b);
}
