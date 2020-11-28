/*
  IMU Classifier

  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU, once enough samples are read, it then uses a
  TensorFlow Lite (Micro) model to try to classify the movement as a known gesture.

  Note: The direct use of C/C++ pointers, namespaces, and dynamic memory is generally
        discouraged in Arduino examples, and in the future the TensorFlowLite library
        might change to make the sketch simpler.

  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.

  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry

  This example code is in the public domain.
*/
// let's get ready to wifiiiiii
#include <WiFiClient.h>
#include <WiFi.h>
#include "environ.h"

const char* ssid     = mySSID;
const char* password = myPASS;

WiFiClient client;

// Add Hue lib
#include <ESPHue.h>

ESPHue myHue = ESPHue(client, myHUEKEY, myHUEIP, 80);

// Add 9DoF library
#include "ICM_20948.h"

// 9DoF sensor interface
ICM_20948_I2C myICM;


#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h>
#include <tensorflow/lite/experimental/micro/micro_error_reporter.h>
#include <tensorflow/lite/experimental/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

#include "model.h"

const float accelerationThresholdG = 2.5; // threshold of significant in G's
const int numSamples = 119;

int samplesRead = numSamples;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::ops::micro::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];

// array to map gesture index to a name
const char* GESTURES[] = {
  "tap",
  "twist"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

void setup() {
  // Initialize Serial port
  Serial.begin(115200);
  while (!Serial)
  {
  };

  // set up wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); 
  
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

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}

void loop() {
  float aX, aY, aZ, gX, gY, gZ;

  // Wait for acceleration to begin gesture sample collection
  while (samplesRead == numSamples)
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

  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
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

      // normalize the IMU data between 0 to 1 and store in the model's
      // input tensor
      tflInputTensor->data.f[samplesRead * 6 + 0] = (aX + 4.0) / 8.0;
      tflInputTensor->data.f[samplesRead * 6 + 1] = (aY + 4.0) / 8.0;
      tflInputTensor->data.f[samplesRead * 6 + 2] = (aZ + 4.0) / 8.0;
      tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
      tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
      tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;

      samplesRead++;

      if (samplesRead == numSamples) {
        // Run inferencing
        TfLiteStatus invokeStatus = tflInterpreter->Invoke();
        if (invokeStatus != kTfLiteOk) {
          Serial.println("Invoke failed!");
          while (1);
          return;
        }

        // Loop through the output tensor values from the model
        for (int i = 0; i < NUM_GESTURES; i++) {
          Serial.print(GESTURES[i]);
          Serial.print(": ");
          Serial.println(tflOutputTensor->data.f[i], 6);
        }

        float tapConfidence = tflOutputTensor->data.f[0]; //0 here is hardcoded, probably bad

        // If it's a tap, toggle a light
        if (tapConfidence > 0.9) 
        {
          int lightID = 14; // CJ Room is 33, Piano Lamp is 14
          Serial.println("Toggling Light!");
          if (myHue.getLightState(lightID) == 1) 
          {
            Serial.println("Hue Light is Currently On, Turning Light Off");
            myHue.setLightPower(lightID, myHue.OFF);
          }
          else // Light lightID is off
          {
            Serial.println("Hue Light  is Currently Off, Turning Light On");
            myHue.setLightPower(lightID, myHue.ON);
          }
        }
        
        Serial.println();
      }
    }
  }
}
