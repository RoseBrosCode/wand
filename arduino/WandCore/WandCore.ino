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
int lightID = 33; // CJ's Room is 33, Zach's Nightstand is 2, Piano Lamp is 14
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

int currentTouchEvent = -1;
int prevTouchEvent = -1;
int lastLogged = -1; // DEBUG ONLY
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up remote debug
////////////////////////////////////////////////////////////////////////////////////
#include <RemoteDebug.h> // https://github.com/JoaoLopesF/RemoteDebug

RemoteDebug Debug;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// JSON Library
////////////////////////////////////////////////////////////////////////////////////
#include <ArduinoJson.h>
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// RGB Converter
////////////////////////////////////////////////////////////////////////////////////
#include "RGBConverter.h"

RGBConverter converter = RGBConverter();
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up wand state variables
////////////////////////////////////////////////////////////////////////////////////
// wand mode definitions
#define POWER 0
#define COLOR 1
int wandMode = POWER; // default on boot
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
  setupTouchSurface(T3);

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
  for (int i = 0; i < 5; i++) {
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

  // Read IMU sensor
  readIMU(imuData);

  // Read the touch surface and handle the event
  // Take immediate action if single tap or double tap; if holding, action happens only if gesture is detected
  prevTouchEvent = currentTouchEvent;
  currentTouchEvent = getTouchEvent();
  // N.B. Single tap is very inconsistent, exhibits frequent false negatives.
  // May be fixed with tuning touch client, but avoid use for now.
  if (currentTouchEvent == SINGLE_TAP) {
    debugI("Single Tap Just Happened");
    if (wandMode == POWER) {                      // power mode, NOT DOING (was: single tap == set light brightness based on "brightness" of color sensed)
      // TODO
    }
    else if (wandMode == COLOR) {                 // color mode, NOT DOING (was: single tap == set color of light based on color sensed)
      // TODO
    }  
  } else if (currentTouchEvent == DOUBLE_TAP) {
    debugI("Double Tap Just Happened");
    // toggle the wand's mode
    if (wandMode == POWER) {                      // power mode, double tap == switch to color mode
      wandMode = COLOR;
      incrementRGBColorloop();                    // resume cycling the onboard RGB through its colorloop
      debugI("Switched to Color Mode");
    } else {                                      // color mode, double tap == switch to power mode
      wandMode = POWER;
      setRGBLED(255, 255, 255);                   // solid white for power mode
      debugI("Switched to Power Mode"); 
    }
  } else if (currentTouchEvent == HOLDING && prevTouchEvent != HOLDING) { // Captures the start of holding
    debugI("Holding Start Just Happened");
    if (wandMode == POWER) {                      // power mode, start holding == toggle color sensor LED
      setColorSensorLED(!getColorSensorLED());
    }
    else if (wandMode == COLOR) {                 // color mode, start holding == set color of light based on color sensed
      // Turn off LED to not interfere with color sensing
      setRGBLED(0, 0, 0);

      // Wait for LED to turn off
      delay(50);

      // read the color sensor
      getRGB(&sensorR, &sensorG, &sensorB);

      // cast rgb to byte for conversions
      byte br = (uint16_t) sensorR;
      byte bg = (uint16_t) sensorG;
      byte bb = (uint16_t) sensorB;

      // NOTE: RGB readings are very dark
      // translate from RGB to HSV, boost S and V, translate back to RGB, then to XY

      // RGB -> HSV
      double hsv[3] = {0.0, 0.0, 0.0};
      converter.rgbToHsv(br, bg, bb, hsv);

      // boost HSV
      double sBoost = 0.2;
      double boostedS = min(hsv[1] + sBoost, 1.0);
      double vBoost = 1.0;
      double boostedV = min(hsv[2] + vBoost, 1.0);

      // HSV (boosted) -> RGB
      byte boostedRGB[3] = {0, 0, 0};
      converter.hsvToRgb(hsv[0], boostedS, boostedV, boostedRGB);
      
      // RGB (boosted) -> XY
      double xy[2] = {0.0, 0.0};
      converter.rgbToCIE1931XY(boostedRGB[0], boostedRGB[1], boostedRGB[2], xy);

      // debug print
      debugI("Original: R: %u, G: %u, B: %u | H: %f, S: %f, V: %f", br, bg, bb, hsv[0], hsv[1], hsv[2]);
      debugI("Boosted: R: %u, G: %u, B: %u | H: %f, S: %f, V: %f | X: %f, Y: %f", boostedRGB[0], boostedRGB[1], boostedRGB[2], hsv[0], boostedS, boostedV, xy[0], xy[1]);

      // set light with hue API - CIE 1931 XY
      myHue.setLight(lightID, myHue.ON, xy);

      // Return LED to color loop
      incrementRGBColorloop();
    }
  }

  // Calculate sum of acceleration to compare to threshold
  float aSum = fabs(imuData[AX]) + fabs(imuData[AY]) + fabs(imuData[AZ]);
  float gSum = fabs(imuData[GX]) + fabs(imuData[GY]) + fabs(imuData[GZ]);

  // Acceleration threshold reached, handle gesture
  if (aSum >= accelerationThresholdG || gSum >= gyroThreshold) {
    // do the gesture predicting
    readAndPredictGesture(imuData, gestureConfidence);
    
    // read target light state
    // only do this here as it's not needed elsewhere and this minimizes polling
    // step 1 of 3 - prepare the raw string from the Hue library
    String rawLightState;
    rawLightState = myHue.getLightInfo(lightID);
    int removeToIdx = rawLightState.indexOf("{") - 1;
    rawLightState.remove(0, removeToIdx);

    // step 2 of 3 - parse the JSON
    StaticJsonDocument<112> filter;
    JsonObject filter_state = filter.createNestedObject("state");
    filter_state["hue"] = true;
    filter_state["on"] = true;
    filter_state["effect"] = true;
    filter_state["bri"] = true;
    filter_state["sat"] = true;
    filter_state["ct"] = true;
    
    StaticJsonDocument<192> doc;
    
    DeserializationError error = deserializeJson(doc, rawLightState, DeserializationOption::Filter(filter));
    
    if (error) {
      debugE("deserializeJson failed: %s", error.c_str());
      return;
    }

    // step 3 of 3 - assign temporary state variables
    JsonObject lightCurrentState = doc["state"];
    long lightCurrentHue = lightCurrentState["hue"]; // 0(red)-65535(red); green = 21845 and blue = 43690
    bool lightCurrentPower = lightCurrentState["on"]; // true = light is on
    const char* lightCurrentEffect = lightCurrentState["effect"]; // "none" or "colorloop"
    int lightCurrentBrightness = lightCurrentState["bri"]; // 1(minimum brightness)-254(maximum brightness)
    int lightCurrentSaturation = lightCurrentState["sat"]; // 0(least saturated; white)-254(most saturated; full color)
    int lightCurrentColorTemp = lightCurrentState["ct"]; // 153-500 (6500K-2000K)
    debugI("Current light properties: Hue = %d, Brightness = %d, Saturation = %d, Temp = %d, effect = %s", lightCurrentHue, lightCurrentBrightness, lightCurrentSaturation, lightCurrentColorTemp, lightCurrentEffect);

    // process actions based on gesture, mode, and touch surface state
    if (gestureConfidence[GESTURE_FLICK] > 0.9) {                       // it's a flick, do flick things!
      debugI("Flick registered. Mode is %d.", wandMode);
      if (currentTouchEvent == HOLDING) {
        if (wandMode == POWER) {                                        // power mode, holding, flick == turn up the brightness
          // TODO
        }
      } else {
        if (wandMode == POWER) myHue.setLightPower(lightID, myHue.ON);  // power mode, NOT holding, flick == turn on the light
        else if (wandMode == COLOR) {                                   // color mode, NOT holding, flick == cycle through pre-defined color favorites
          // TODO
 
          // Desired XY color values
          // 0.6779, 0.2968 - Red
          // 0.6464, 0.3438 - Orange
          // 0.5087, 0.4601 - Yellow
          // 0.214, 0.709 - Green
          // 0.1825, 0.4448 - Teal
          // 0.1471, 0.149 - Blue
          // 0.1996, 0.1041 - Purple
          // 0.513, 0.2236 - Pink
        }
      }      
    }
    else if (gestureConfidence[GESTURE_TWIST] > 0.9) {                  // it's a twist, do twist things!    
      debugI("Twist registered. Mode is %d", wandMode);
      if (currentTouchEvent == HOLDING) {
        if (wandMode == POWER) {                                        // power mode, holding, twist == turn down the brightness
          // TODO
        }
      } else {
        if (wandMode == POWER) myHue.setLightPower(lightID, myHue.OFF); // power mode, NOT holding, twist == turn off the light
        else if (wandMode == COLOR) {                                   // color mode, NOT holding, twist == toggle colorloop on light
          if (String(lightCurrentEffect) == "colorloop") myHue.setLightColorloop(lightID, myHue.OFF);
          else myHue.setLightColorloop(lightID, myHue.ON);
        }       
      }      
    }
  }

  // facilitates remote debugging
  Debug.handle();
}
