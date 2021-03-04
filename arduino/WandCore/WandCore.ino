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

// Defines the light the wand is hard-coded to. 
int lightID = 14; // CJ's Room is 33, Piano Lamp is 14
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
#include "TouchSurfaceClient.h"

int lastLogged = -1; // DEBUG ONLY
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
// wand mode definitions
#define POWER 0
#define COLOR 1
int wandMode = POWER; // default on boot
bool isLightColorloopOn = false; // this holds the state of the *target light bulb's* colorloop
////////////////////////////////////////////////////////////////////////////////////


void setup() 
{
  // Initialize Serial port
  Serial.begin(115200);
  while (!Serial) {};

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
      int numProgress = 100 * progress;                       // this step enables the next step
      int percent = numProgress / total;                      // get progress as a percentage - int will be truncated
      if (percent % 10 == 0 && percent > otaProgress) {       // every 10 percent of progress, print the percentage, and don't print the same percentage repeatedly
        otaProgress = percent;                                // allows for only printing the percentage once, since there will be numerous calls where percent % 10 results in the same number
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

  // Configure touch surface
  setupTouchSurface(T9);

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

  // DEBUG ONLY
  // testDevices();
}

/*
 * Exercises attached devices 
 */
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
  // Check for OTA
  ArduinoOTA.handle();

  // If in Color Mode, increment the RGB LED in the loop
  if (wandMode == COLOR) {
    incrementRGBColorloop();
  }

  // Read the touch surface and handle the event
  int currentTouchEvent = getTouchEvent();
  switch (currentTouchEvent)
  {
    case SINGLE_TAP:
      // trigger color sensor
      debugI("single tap happened");
      
      break;
    case DOUBLE_TAP:
      debugI("double tap happened");
      // change mode
      if (wandMode == POWER) {
        wandMode = COLOR;
        incrementRGBColorloop(); 
        debugI("Switched to Color Mode");
      } else {
        wandMode = POWER;
        setRGBLED(255, 255, 255); // white for power mode
        debugI("Switched to Power Mode"); 
      }      
      
      break;
    case HOLDING:
      // TODO adjust brightness or color depending on mode
      if ((millis() - lastLogged) >= 500) { // DEBUG ONLY, throttling log output
        debugI("touch is being held");
        lastLogged = millis();
      }
      
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
    
    if (gestureConfidence[GESTURE_FLICK] > 0.9) { // it's a flick, do flick things!
      debugI("Flick registered. Mode is %d.", wandMode);
      if (wandMode == POWER) myHue.setLightPower(lightID, myHue.ON); // turn on the light
      else if (wandMode == COLOR) { // color mode, cycle through color presets
        
      }
      
    }
    else if (gestureConfidence[GESTURE_TWIST] > 0.9) { // it's a twist, do twist things!    
      debugI("Twist registered. Mode is %d", wandMode);
      if (wandMode == POWER) myHue.setLightPower(lightID, myHue.OFF); // turn off the light
      else if (wandMode == COLOR) { // toggle colorloop on bulb
        if (isLightColorloopOn) {
          myHue.setLightColorloop(lightID, myHue.OFF);
          isLightColorloopOn = false;
        }
        else {
          myHue.setLightColorloop(lightID, myHue.ON);
          isLightColorloopOn = true;
        }
      }
      
    }
  }

  // facilitates remote debugging
  Debug.handle();
}
