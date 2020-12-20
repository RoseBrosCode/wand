/*
  Gesture Predicition

  Uses a trained TensorFlow Lite model to classify input gestures.

  Adapted from: https://blog.arduino.cc/2019/10/15/get-started-with-machine-learning-on-arduino/
  Originally Created by Don Coleman, Sandeep Mistry
  Originally Modified by Dominic Pajak, Sandeep Mistry
*/

#include "GesturePrediction.h"

// Global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// Pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::ops::micro::AllOpsResolver tflOpsResolver;

// TFLM variables used for inference
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize];

// Array to map gesture index to a name
const char* GESTURES[] = {
  "flick",
  "twist"
};

// Number of samples per gesture
const int numSamples = 119;

// Counter of samples read
int samplesRead = numSamples;

// Flag that enables debug printing
#define DEBUG_GESTURE_PREDICTION 1

/*
 * Loads the model file and instantiates input and output tensors.
 */
void setupGesturePrediction()
{
  // Get the TFL representation of the model byte array
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

/*
 * Collects IMU samples and runs inference to predict which gesture was performed.
 */
void readAndPredictGesture(float imuData[6], float confidence[NUM_GESTURES])
{
  // Reset sample counter
  samplesRead = 0;

  // Collect all samples
  while (samplesRead < numSamples) {
    // Read IMU sensor
    readIMU(imuData);

    // Normalize the IMU data between 0 to 1,
    // store in the model's input tensor
    tflInputTensor->data.f[samplesRead * 6 + 0] = (imuData[AX] + 4.0) / 8.0;
    tflInputTensor->data.f[samplesRead * 6 + 1] = (imuData[AY] + 4.0) / 8.0;
    tflInputTensor->data.f[samplesRead * 6 + 2] = (imuData[AZ] + 4.0) / 8.0;
    tflInputTensor->data.f[samplesRead * 6 + 3] = (imuData[GX] + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 4] = (imuData[GY] + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 5] = (imuData[GZ] + 2000.0) / 4000.0;

    // Increase counter
    samplesRead++;
  }

  // Run inferencing
  TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  if (invokeStatus != kTfLiteOk) {
    Serial.println("Inferencing Invoke failed!");
    while (1);
    return;
  }

  // Iterate through output
  for (int i = 0; i < NUM_GESTURES; i++) {
    // Print the output tensor values from the model
    if (DEBUG_GESTURE_PREDICTION) {
      Serial.print(GESTURES[i]);
      Serial.print(": ");
      Serial.println(tflOutputTensor->data.f[i], 6);
    } 

    // Update confidence array with prediction
    confidence[i] = tflOutputTensor->data.f[i];
  }
}
