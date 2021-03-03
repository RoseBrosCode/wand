/*
  Capacitive Touch Client

  Manages a Touch Surface Input
*/

#include "CAPClient.h"

// touch threshold
const int touchThold = 50; //above means not touched

// Touch surface timing variables
int debounce = 20;                // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;                  // max ms between clicks for a double click event
int holdTime = 1000;              // ms hold period: how long to wait for tap+hold event

// Touch surface variables
boolean isTouched = false;        // value read from touch surface
boolean wasTouchedLast = false;   // buffered value of the button's previous state
boolean DCwaiting = false;        // whether we're waiting for a double tap (down)
boolean DConUp = false;           // whether to register a double tap on next release, or whether to wait and tap
boolean singleOK = true;          // whether it's OK to do a single tap
unsigned long downTime = -1;      // time the surface was tapped
unsigned long upTime = -1;        // time the surface was released
boolean ignoreUp = false;         // whether to ignore the surface release because the tap+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already

bool checkIfTouched(int pin) {
  int touchVal = touchRead(pin);
  if (touchVal > touchThold) return false;
  else return true;
}

int getTouchEvent(int pin) {   
  int event = 0;
  isTouched = checkIfTouched(pin);
  
  // Surface is just touched
  if (isTouched == true && wasTouchedLast == false && (millis() - upTime) > debounce)
  {
    downTime = millis();
    ignoreUp = false;
    // waitForUp = false; // may be unnecessary?
    singleOK = true;
    holdEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
    else  DConUp = false;
    DCwaiting = false;
  }
  
  // Surface no longer being touched
  else if (isTouched == false && wasTouchedLast == true && (millis() - downTime) > debounce)
  {       
    if (not ignoreUp)
    {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else
      {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  
  // Test for single tap event: DCgap expired
  if (isTouched == false && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
  {
    event = 1;
    DCwaiting = false;
  }
  
  // Test for hold
  if (isTouched == true && (millis() - downTime) >= holdTime)
  {
    event = 3;    
    if (not holdEventPast)
    {
      // waitForUp = true; // may be unnecessary?
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      holdEventPast = true;
    }
  }
  wasTouchedLast = isTouched;
  return event;
}
