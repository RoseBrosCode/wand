/*
  IMU Client

  Manages the SparkFun 9DoF IMU Breakout - ICM-20948 (Qwiic).
*/

#include "IMUClient.h"

// 9DoF sensor interface
ICM_20948_I2C myICM;

// Unique I2C port
TwoWire i2cPort = TwoWire(0);

// Flag that enables debug printing
#define DEBUG_IMU_CLIENT 1

/*
 * Initializes the I2C connection to the 9DoF IMU sensor, and configures its output values.
 */
void setupIMU()
{
  // Initialize I2C
  i2cPort.begin();
  i2cPort.setClock(400000);

  // Initialize 9DoF
  myICM.begin(i2cPort, 1); // 1 is the last bit of the default I2C address
  while (myICM.status != ICM_20948_Stat_Ok) {
     Serial.println(myICM.statusString());
     delay(500);
  }

  // Configure 9DoF accelerometer to +-4G and gyroscope +=2000deg/s
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;
  myFSS.g = dps2000;
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  Serial.println("Successfully set up IMU");
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

  // Debug print values read
  if (DEBUG_IMU_CLIENT) {
    Serial.print("AX: ");
    Serial.print(output[AX], 6);
    Serial.print(" AY: ");
    Serial.print(output[AY], 6);
    Serial.print(" AZ: ");
    Serial.print(output[AZ], 6);
    Serial.print(" GX: ");
    Serial.print(output[GX], 6);
    Serial.print(" GY: ");
    Serial.print(output[GY], 6);
    Serial.print(" GZ: ");
    Serial.println(output[GZ], 6);
  }
}
