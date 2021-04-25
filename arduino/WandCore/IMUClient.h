/*
  IMU Client

  Manages the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic).
*/

#ifndef IMU_CLIENT
#define IMU_CLIENT

// Add 9DoF library
#include "ICM_20948.h"

// Definitions of IMU data indices
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

// Initializes the I2C connection to the 9DoF IMU sensor, and configures its output values.
void setupIMU();

// Reads acceleration and gyroscope data from the IMU sensor.
void readIMU(float output[6]);

#endif
