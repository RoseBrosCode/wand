/*
  Capacitive Touch Surface Client

  Manages a Touch Surface Input
*/

#include "TouchSurfaceClient.h"

// Pin the touch surface is attached to
int touchPin;

// Touch surface constants
const int touchThreshold = 65;              // readings above this value mean the sensor is not being touched
const int debounce = 20;                    // ms debounce period to prevent flickering when touching or stopping of touching the surface
const int doubleTapGap = 250;               // max ms between taps for a double tap event
const int holdTime = 1000;                  // ms hold period: how long to wait for hold event; TODO experiment if this can shorten

// Touch surface state variables
boolean isTouched = false;                  // whether or not the touch surface is being touched during this loop
boolean wasTouchedLast = false;             // whether or not the touch surface was touched last loop
boolean doubleTapWaiting = false;           // true if a double tap may be pending
boolean doubleTapOnUp = false;              // true if double tap conditions have been met but the second tap is still being touched
unsigned long downTime = -1;                // most recent time when the surface began to be touched
unsigned long upTime = -1;                  // most recent time when the surface was stopped being touched
boolean holdEventOngoing = false;           // true if a hold event started on a previous iteration and another new touch has not occured

/*
 * Establishes the pin that the touch surface is attached to
 */
void setupTouchSurface(int pin)
{
  touchPin = pin;
  Serial.println("Successfully set up touch surface.");
}

/* 
 * Reads the touch sensor and returns if the surface is currently is being touched
 */
bool checkIfTouched()
{
  int touchVal = touchRead(touchPin);
  if (touchVal > touchThreshold) return false;
  else return true;
}

/* 
 * getTouchEvent
 * Returns an int representing the UI action a user has taken with the touch surface 
 * 
 * NO_INTERACTION = not interacting with the surface
 * SINGLE_TAP = user single tapped
 * DOUBLE_TAP = user double tapped
 * HOLDING = user tapped and is holding
 */
int getTouchEvent()
{   
  int event = NO_INTERACTION; // if none of the below cases set a different event, then there's no interaction to report yet
  isTouched = checkIfTouched();

  // Used debounced conditional for event detection
  bool debouncedTouched = isTouched && (millis() - upTime) > debounce;
  bool debouncedUntouched = not isTouched && (millis() - downTime) > debounce;
  
  // Surface is just touched
  if (isTouched == true                     // the surface is sensing a touch
  && wasTouchedLast == false                // last time through it did NOT sense a touch
  && (millis() - upTime) > debounce) {      // debounce
    downTime = millis();                    // set the new time of the most recent touch    
    if ((millis() - upTime) < doubleTapGap  // the time since the previous release is less than the double tap gap
    && doubleTapOnUp == false               // we aren't already planning to send a double tap event once no longer being touched
    && doubleTapWaiting == true) {          // we were waiting to see if it was a double tap (and not just a single tap)
      doubleTapOnUp = true;                 // this is a double tap! send the event when no longer being touched
      doubleTapWaiting = false;             // reset this since we're no longer waiting and seeing
    } else doubleTapOnUp = false;          // otherwise it's not a double tap
  }

  // Surface no longer being touched        
  else if (isTouched == false               // the surface is NOT sensing a touch
  && wasTouchedLast == true                 // last time through it DID sense a touch
  && (millis() - downTime) > debounce) {    // debounce
    upTime = millis();                      // set new time of the most recent stopping of touching
    
    if (holdEventOngoing) {                 
      holdEventOngoing = false;             // if no longer being touched, then the hold event is no longer happening    
    } else {                                // process as release of a tap
      if (doubleTapOnUp == false) {         // not yet a confirmed double tap
        doubleTapWaiting = true;            // single tap detected, might be a double tap, this tells us to wait and see in the next loops
      }
      else {                                // we know it's a double tap and the second release has occured, so send the event and reset
        event = DOUBLE_TAP;                 // set the double tap event
        doubleTapOnUp = false;              // no longer primed for a double tap because we just set the event
      }
    }
  }
  
  // Test for single tap event
  if (isTouched == false                    // the surface is NOT sensing a touch
  && (millis() - upTime) >= doubleTapGap    // it's been too long for it to be a double tap
  && doubleTapWaiting == true) {            // there was a real initial tap that led to waiting for a double tap (not just a hold event ending)
    event = SINGLE_TAP;                     // set the single tap event
    doubleTapWaiting = false;               // double tap window missed so we're not waiting for it anymore
  }

  // Test for hold event
  if ((isTouched == true ||                 // the surfaces is sensing a touch
       debouncedTouched || not debouncedUntouched )  // ensure HOLDING events persist false negatives for isTouched
  && (millis() - downTime) >= holdTime) {   // the touch has been continuous for longer than the hold time
    event = HOLDING;                        // set the hold event - note this gets sent continuously, not just when the hold starts

    // first time through, set vars to ensure clean completion of hold
    if (not holdEventOngoing) {             // previously this was not a hold event
      doubleTapOnUp = false;                // might have been a double tap leading into the hold, but that should be canceled now
      doubleTapWaiting = false;             // no longer waiting for a double tap as we're in a hold
      holdEventOngoing = true;              // we're in a hold event now
    }
  }
  wasTouchedLast = isTouched;               // store the state for the next pass
  return event;
}
