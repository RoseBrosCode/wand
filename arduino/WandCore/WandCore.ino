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
int otaProgress = 0;
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
// Touch Surface
////////////////////////////////////////////////////////////////////////////////////
#include "CAPClient.h"

// Pin used for capacitive touch button
int touchPin = T9;
int logDelay = 0;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up remote debug
////////////////////////////////////////////////////////////////////////////////////
#include <RemoteDebug.h>        //https://github.com/JoaoLopesF/RemoteDebug

RemoteDebug Debug;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up state
////////////////////////////////////////////////////////////////////////////////////
int wandMode = 1; // 1 = power/brightness, 2 = color
int rainbowSection = 1; // 1 through 6, one for each section of https://academe.co.uk/wp-content/uploads/2012/04/451px-HSV-RGB-comparison.svg_.png
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
      debugI("Start updating %d", type);
    })
    .onEnd([]() {
      debugI("OTA upload complete. Rebooting...");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      int bigprog = 100 * progress;
      int percent = bigprog / total;
      int permod = percent % 10;
      if (percent % 10 == 0 && percent > otaProgress) 
      {
        otaProgress = percent;
        debugI("OTA upload is %d percent complete", percent);
      }
    })
    .onError([](ota_error_t error) {
      debugE("OTA Error[%d]: ", error);
      if (error == OTA_AUTH_ERROR) debugE("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) debugE("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) debugE("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) debugE("Receive Failed");
      else if (error == OTA_END_ERROR) debugE("End Failed");
    });

  ArduinoOTA.begin();

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

  // Use the built-in LED as visual feedback
  pinMode(LED_BUILTIN, OUTPUT);

  // Flash onboard and RGB LEDs to indicate end of setup
  digitalWrite(LED_BUILTIN, HIGH);
  setRGBLED(255, 0, 0);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  setRGBLED(0, 0, 0);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  setRGBLED(0, 0, 255);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  setRGBLED(0, 0, 0);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  setRGBLED(0, 255, 0);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  setRGBLED(255, 255, 255); // white for power mode, which is default

  // testDevices();
}

//void testDevices()
//{
//  // Test RGB LED
//  setRGBLED(255, 0, 0);
//  delay(1000);
//  setRGBLED(0, 255, 0);
//  delay(1000);
//  setRGBLED(0, 0, 255);
//  delay(1000);
//  setRGBLED(0, 0, 0);
//  delay(1000);
//
//  // Test color sensor
//  setColorSensorLED(true);
//  delay(1000);
//  setColorSensorLED(false);
//  for (int i = 0; i < 5; i++)
//  {
//    getRGB(&sensorR, &sensorG, &sensorB);
//    delay(200);
//  }
//}

void loop() 
{
  ArduinoOTA.handle();

  // If in Color Mode, increment the RGB LED in the loop
  if (wandMode == 2)
  {
    // coming next
  }

  // Read the touch surface
  int currentTouchEvent = getTouchEvent(touchPin);

  switch (currentTouchEvent) {
    case 1: // single tap
      // trigger color sensor
      debugI("single tap happened");
      
      break;
    case 2: // double tap
      // change mode
      if (wandMode == 1)
      {
        wandMode = 2;
        setRGBLED(255, 0, 0); // colorloop starts on red
        rainbowSection = 1; // colorloop starts at the beginning of the rainbow
        debugI("Switched to Color Mode");
      }
      else
      {
        wandMode = 1;
        setRGBLED(255, 255, 255); // white for power mode
        debugI("Switched to Power Mode"); 
      }      
      
      break;
    case 3: // tap and hold
      // adjust brightness or color depending on mode
      if (logDelay == 0 || logDelay == 100)
      {
        debugI("touch is being held");
        logDelay = 0;
      }
      logDelay++;
      
      break;
  }
  
  // Read IMU sensor
  readIMU(imuData);

  // Calculate sum of acceleration and compare to threshold
  float aSum = fabs(imuData[AX]) + fabs(imuData[AY]) + fabs(imuData[AZ]);
  float gSum = fabs(imuData[GX]) + fabs(imuData[GY]) + fabs(imuData[GZ]);

  // Acceleration threshold reached, predict gesture
  if (aSum >= accelerationThresholdG || gSum >= gyroThreshold) {
    readAndPredictGesture(imuData, gestureConfidence);
    int capVal = touchRead(touchPin);
    
    // If it's a flick, turn the target light on. Twist, turn it off.
    int lightID = 14; // CJ Room is 33, Piano Lamp is 14
    if (gestureConfidence[GESTURE_FLICK] > 0.9) 
    {
      debugI("Flick! Light turning on.");
      debugI("Touch Value is %d.", capVal);
      myHue.setLightPower(lightID, myHue.ON);
    }
    else if (gestureConfidence[GESTURE_TWIST] > 0.9)
    {
      debugI("Twist! Light turning off.");
      debugI("Touch Value is %d.", capVal);
      myHue.setLightPower(lightID, myHue.OFF);
    }
  }

  Debug.handle();
}
