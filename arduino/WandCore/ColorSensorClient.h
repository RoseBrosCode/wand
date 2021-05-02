/*
 * Color Sensor Client
 * 
 * Manages the RGB Color Sensor with IR filter and White LED - TCS34725
 */

#ifndef COLOR_SENSOR_CLIENT
#define COLOR_SENSOR_CLIENT

#include "Adafruit_TCS34725.h"

// Connects to the color sensor over I2C.
void setupColorSensor(int ledPin);

// Reads RGB data from the color sensor.
void getRGB(float *r, float *g, float *b);

// Gets the sensor's LED value.
bool getColorSensorLED();

// Sets the sensor's LED.
void setColorSensorLED(bool val);

#endif
