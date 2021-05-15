/*
  Gesture Predicition

  Uses a trained TensorFlow Lite model to classify input gestures.

  Adapted from: https://blog.arduino.cc/2019/10/15/get-started-with-machine-learning-on-arduino/
  Originally Created by Don Coleman, Sandeep Mistry
  Originally Modified by Dominic Pajak, Sandeep Mistry
*/


#ifndef GESTURE_PREDICTION
#define GESTURE_PREDICTION

// Depends on IMUClient
#include "IMUClient.h"

// Trained model file
#include "model_final3.h"


// TensorFlow Lite dependencies
#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h>
#include <tensorflow/lite/experimental/micro/micro_error_reporter.h>
#include <tensorflow/lite/experimental/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

// Definitions of gesture indices
#define GESTURE_FLICK 0
#define GESTURE_TWIST 1

// Number of gestures
#define NUM_GESTURES 2

// Loads the model file and instantiates input and output tensors.
void setupGesturePrediction();

// Collects IMU samples and runs inference to predict which gesture was performed.
void readAndPredictGesture(float imuData[6], float confidence[NUM_GESTURES]);

#endif
