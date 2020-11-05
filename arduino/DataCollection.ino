// Add Arduino library for VSCode IntelliSense
#include <Arduino.h>

// Add 9DoF library
#include "ICM_20948.h"

// 9DoF sensor interface
ICM_20948_I2C myICM;

// Run once on device startup/reset
void setup()
{
    // Initialize Serial port
    Serial.begin(115200);
    while (!Serial)
    {
    };

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);

    // Initialize 9DoF
    myICM.begin(Wire, 1); // 1 is the last bit of the default I2C address
    while (myICM.status != ICM_20948_Stat_Ok)
    {
        delay(500);
    }

    // Configure 9DoF accelerometer to +-4G and gyroscope +=2000deg/s
    ICM_20948_fss_t myFSS;
    myFSS.a = gpm4;
    myFSS.g = dps2000;
    myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

    // Use the built-in LED as visual feedback
    pinMode(LED_BUILTIN, OUTPUT);

    // Double flash to indicate end of setup
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);

    // Print header of CSV file
    Serial.println("aX,aY,aZ,gX,gY,gZ");
}

// The absolute acceleration threshold to begin IMU values
//  as part of a gesture
const float accelerationThresholdG = 2.5;

// The number of IMU samples ("duration") per gesture
const int numSamplesPerGesture = 119;

// Counter for measuring recorded samples per gesture
int samplesRead = numSamplesPerGesture;

void loop()
{
    // IMU acceleration and gyroscope values
    float aX, aY, aZ, gX, gY, gZ;

    // Wait for acceleration to begin gesture sample collection
    while (samplesRead == numSamplesPerGesture)
    {
        if (myICM.dataReady()) 
        {
            // Read the sensor
            myICM.getAGMT();

            // Read acceleration values and convert mG to G
            aX = myICM.accX() / 1000;
            aY = myICM.accY() / 1000;
            aZ = myICM.accZ() / 1000;

            // Calculate sum of acceleration and compare to threshold
            float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
            if (aSum >= accelerationThresholdG) 
            {
                // Start a new gesture recording by resetting sample counter
                samplesRead = 0;
                break;
            }
        }
    }

    // Collect a new gesture
    while (samplesRead < numSamplesPerGesture)
    {
        if (myICM.dataReady())
        {
            // Read the sensor
            myICM.getAGMT();

            // Read acceleration values and convert mG to G
            aX = myICM.accX() / 1000;
            aY = myICM.accY() / 1000;
            aZ = myICM.accZ() / 1000;

            // Read gyroscope values
            gX = myICM.gyrX();
            gY = myICM.gyrY();
            gZ = myICM.gyrZ();

            // Increase sample counter
            samplesRead++;

            // Print the data in expected CSV format
            Serial.print(aX, 3);
            Serial.print(',');
            Serial.print(aY, 3);
            Serial.print(',');
            Serial.print(aZ, 3);
            Serial.print(',');
            Serial.print(gX, 3);
            Serial.print(',');
            Serial.print(gY, 3);
            Serial.print(',');
            Serial.print(gZ, 3);
            Serial.println();

            // Add an empty line after the last sample
            if (samplesRead == numSamplesPerGesture)
            {
                Serial.println();
            }
        }
    }

    // Triple flash to indicate recording done
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}
