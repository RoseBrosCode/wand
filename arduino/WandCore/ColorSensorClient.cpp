/*
 * Color Sensor Client
 * 
 * Manages the RGB Color Sensor with IR filter and White LED - TCS34725
 */

#include "ColorSensorClient.h"

// Color sensor library object
Adafruit_TCS34725 myTCS = Adafruit_TCS34725();

// Pin for controlling the LED
int ledPin;

// Flag that enables debug printing
#define DEBUG_COLOR_SENSOR_CLIENT 1

/*
 * Initializes the I2C connection to the color sensor.
 */
void setupColorSensor(int pin)
{
  if (!myTCS.begin()) {
    Serial.println("Unable to connect to color sensor.");
    while (1);
  }

  // Setup LED pin
  ledPin = pin;
  pinMode(ledPin, OUTPUT);

  Serial.println("Successfully set up color sensor.");
}

/*
 * Reads from the color sensor.
 */
void getRGB(float *r, float *g, float *b)
{
  // Maintain state of LED when called.
  bool stateOnCall = digitalRead(ledPin);
  
  // Turn LED on for measurement
  setColorSensorLED(true);

  // Wait for LED to turn on
  delay(200);
  
  // Read color sensor
  myTCS.getRGB(r, g, b);

  // Turn LED to previous state
  setColorSensorLED(stateOnCall);

  // Debug print the value read
  if (DEBUG_COLOR_SENSOR_CLIENT) {
    Serial.print("R: ");
    Serial.print(*r);
    Serial.print(" G: ");
    Serial.print(*g);
    Serial.print(" B: ");
    Serial.println(*b);
  }
}

/*
 * Gets the sensor's LED value.
 */
bool getColorSensorLED()
{
  return digitalRead(ledPin);
}

/*
 * Sets the sensor's LED.
 */
void setColorSensorLED(bool val)
{
  digitalWrite(ledPin, val);
}
