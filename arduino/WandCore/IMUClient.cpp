/*
  IMU Client

  Manages the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic).
*/

#include "IMUClient.h"

// 9DoF sensor interface
ICM_20948_I2C myICM;

/*
 * Initializes the I2C connection to the 9DoF IMU sensor, and configures its output values.
 */
void setupIMU()
{
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);

  // Initialize 9DoF
  myICM.begin(Wire, 1); // 1 is the last bit of the default I2C address
  while (myICM.status != ICM_20948_Stat_Ok) delay(500);

  // Configure 9DoF accelerometer to +-4G and gyroscope +=2000deg/s
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;
  myFSS.g = dps2000;
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
}

/*
 * Reads acceleration and gyroscope data from the IMU sensor.
 */
void readIMU(float output[6])
{
  // Wait for data to be ready
  while(!myICM.dataReady()){}

  // Read the sensor
  myICM.getAGMT();

  // Read acceleration values and convert mG to G
  output[AX] = myICM.accX() / 1000;
  output[AY] = myICM.accY() / 1000;
  output[AZ] = myICM.accZ() / 1000;

  // Read gyroscope values
  output[GX] = myICM.gyrX();
  output[GY] = myICM.gyrY();
  output[GZ] = myICM.gyrZ();
}
