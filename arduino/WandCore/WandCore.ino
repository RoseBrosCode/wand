/*
  Wand Core

  Contains variable definitions and main functions the Project Sparkle wand.
*/
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
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Hue Client
////////////////////////////////////////////////////////////////////////////////////
// Add Hue lib
#include <ESPHue.h>

// Hue lib singleton
ESPHue myHue = ESPHue(client, myHUEAPIKEY, myHUEBRIDGEIP, 80);
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// IMU Client
////////////////////////////////////////////////////////////////////////////////////
#include "IMUClient.h"

// Buffer to populate with IMU data
float imuData[6];
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Gesture Prediction
////////////////////////////////////////////////////////////////////////////////////
#include "GesturePrediction.h"

// Buffer to populate with gesture prediction confidence data
float gestureConfidence[NUM_GESTURES];

// Threshold for gesture detection in G's
const float accelerationThresholdG = 5.25;
const float gyroThreshold = 1250;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Color Sensor Client
////////////////////////////////////////////////////////////////////////////////////
#include "ColorSensorClient.h"

// Color values read from the color sensor
float sensorR;
float sensorG;
float sensorB;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// RGB LED Client
////////////////////////////////////////////////////////////////////////////////////
#include "LEDClient.h"
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Touch Button
////////////////////////////////////////////////////////////////////////////////////

// Pin used for capacitive touch button
// NOTE: Touch pin T8 refers to physical pin 32, NOT 33 as it references in most
//  documentation. This is due to some pin assignment issue, see:
//  https://randomnerdtutorials.com/esp32-touch-pins-arduino-ide/
int touchPin = T8;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Touch Button
////////////////////////////////////////////////////////////////////////////////////
#include <RemoteDebug.h>        //https://github.com/JoaoLopesF/RemoteDebug

RemoteDebug Debug;
////////////////////////////////////////////////////////////////////////////////////

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


  // Set Up OTA
  ArduinoOTA.setHostname(HOST_NAME);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
//      debugI("Start updating " + type); // also send to remote logging
    })
    .onEnd([]() {
      Serial.println("\nEnd");
//      debugI("/nEnd"); // also send to remote logging
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
//      debugI("Progress: %u%%\r", (progress / (total / 100))); // also send to remote logging
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  
// // Below may not be needed, ArduinoOTA.setHostname may do it.
//  if (MDNS.begin(HOST_NAME)) {
//      Serial.print("MDNS responder started. Hostname -> ");
//      Serial.println(HOST_NAME);
//  }

  // Configure Remote Debug
  MDNS.addService("telnet", "tcp", 23);
  Debug.begin(HOST_NAME); // Initialize the WiFi server
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true); // Colors
  
  // Configure color sensor
  setupColorSensor(14);

  // Turn onboard LED off
  setColorSensorLED(false);

  // Configure 9DoF IMU
  setupIMU();

  // Configure TFLM
  setupGesturePrediction();

  // Configure RGB LED
  setupRGBLED(21, 17, 16);

  // Turn RGB LED off
  setRGBLED(0, 0, 0);

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

  // testDevices();
}

void testDevices()
{
  // Test RGB LED
  setRGBLED(255, 0, 0);
  delay(1000);
  setRGBLED(0, 255, 0);
  delay(1000);
  setRGBLED(0, 0, 255);
  delay(1000);
  setRGBLED(0, 0, 0);
  delay(1000);

  // Test color sensor
  setColorSensorLED(true);
  delay(1000);
  setColorSensorLED(false);
  for (int i = 0; i < 5; i++)
  {
    getRGB(&sensorR, &sensorG, &sensorB);
    delay(200);
  }
}

void loop() 
{
  ArduinoOTA.handle();
  
  // Read IMU sensor
  readIMU(imuData);

  // Calculate sum of acceleration and compare to threshold
  float aSum = fabs(imuData[AX]) + fabs(imuData[AY]) + fabs(imuData[AZ]);
  float gSum = fabs(imuData[GX]) + fabs(imuData[GY]) + fabs(imuData[GZ]);

  // Acceleration threshold reached, predict gesture
  if (aSum >= accelerationThresholdG || gSum >= gyroThreshold) {
    readAndPredictGesture(imuData, gestureConfidence);
    
    // If it's a flick, turn the target light on. Twist, turn it off.
    int lightID = 14; // CJ Room is 33, Piano Lamp is 14
    if (gestureConfidence[GESTURE_FLICK] > 0.9) 
    {
      Serial.println("Flick! Light turning on.");
      debugI("Flick! Light turning on.");

      myHue.setLightPower(lightID, myHue.ON);
    }
    else if (gestureConfidence[GESTURE_TWIST] > 0.9)
    {
      Serial.println("Twist! Light turning off.");
      debugI("Twist! Light turning off.");

      myHue.setLightPower(lightID, myHue.OFF);
    }
  }

  Debug.handle();
}
