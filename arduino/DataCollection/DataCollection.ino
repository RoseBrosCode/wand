#define HOST_NAME "sparklewand"
////////////////////////////////////////////////////////////////////////////////////
// WiFi Client
////////////////////////////////////////////////////////////////////////////////////
#include <WiFiClient.h>
#include <WiFi.h>
#include "environ.h"

// Connects to WiFi
WiFiClient client;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// mDNS
////////////////////////////////////////////////////////////////////////////////////
#include <DNSServer.h>
#include <ESPmDNS.h>
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// OTA updating
////////////////////////////////////////////////////////////////////////////////////
#include <ArduinoOTA.h>
int otaProgress = 0;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up remote debug
////////////////////////////////////////////////////////////////////////////////////
#include <RemoteDebug.h>        //https://github.com/JoaoLopesF/RemoteDebug

RemoteDebug Debug;
////////////////////////////////////////////////////////////////////////////////////

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

    // Connect to WiFi
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(mySSID);
    WiFi.begin(mySSID, myPASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  
  
    // Set Hostname (for OTA and Debug)
    ArduinoOTA.setHostname(HOST_NAME);
  
    // Configure Remote Debug
    MDNS.addService("telnet", "tcp", 23);
    Debug.begin(HOST_NAME); // Initialize the WiFi server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
    Debug.showColors(true); // Colors 
    Debug.setSerialEnabled(true); // Forwards all logs to serial
  
    // Configure OTA
    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        debugI("Start updating %s", type);
      })
      .onEnd([]() {
        debugI("OTA upload complete. Rebooting...");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        int numProgress = 100 * progress;                       // this step enables the next step
        int percent = numProgress / total;                      // get progress as a percentage - int will be truncated
        if (percent % 10 == 0 && percent > otaProgress) {       // every 10 percent of progress, print the percentage, and don't print the same percentage repeatedly
          otaProgress = percent;                                // allows for only printing the percentage once, since there will be numerous calls where percent % 10 results in the same number
          debugI("OTA upload is %d percent complete", percent);
        }
      })
      .onError([](ota_error_t error) {
        debugE("OTA Error[%s]: ", error);
        if (error == OTA_AUTH_ERROR) debugE("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) debugE("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) debugE("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) debugE("Receive Failed");
        else if (error == OTA_END_ERROR) debugE("End Failed");
      });
  
    ArduinoOTA.begin();

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

}

// The absolute acceleration threshold to begin IMU values
//  as part of a gesture
const float accelerationThresholdG = 5.25;
const float gyroThreshold = 1250;

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
        // Check for OTA
        ArduinoOTA.handle();
    
        // facilitates remote debugging
        Debug.handle();
        
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

            // Calculate sum of acceleration and compare to threshold
            float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
            float gSum = fabs(gX) + fabs(gY) + fabs(gZ);
            if (aSum >= accelerationThresholdG || gSum >= gyroThreshold) 
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
          
            debugI(",%f,%f,%f,%f,%f,%f", aX, aY, aZ, gX, gY, gZ);

            // Add an empty line after the last sample
            if (samplesRead == numSamplesPerGesture)
            {
                debugI("");
            }
        }
    }    
}
