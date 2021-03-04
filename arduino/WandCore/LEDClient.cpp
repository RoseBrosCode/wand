/*
  LED Client

  Manages a common cathode RGB LED.
*/

#include "LEDClient.h"

#define CHAN_0 0
#define CHAN_1 1
#define CHAN_2 2

// RGB Colorloop state vars
int RGBColorloopSection = 0; // this holds the state of where in the 6 rainbow sections (0-5) the colorloop is
int RGBColorloopVar = 0; // this holds the state of the RGB LED colorloop variable

// Initializes RGB pins.
void setupRGBLED(int redPin, int greenPin, int bluePin)
{
  // Assign PWM channels to GPIO pins.
  ledcAttachPin(redPin, 0);
  ledcAttachPin(greenPin, 1);
  ledcAttachPin(bluePin, 2);

  // Initialize PWM channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(CHAN_0, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(CHAN_1, 12000, 8);
  ledcSetup(CHAN_2, 12000, 8);

  Serial.println("Successfully set up RGB LED.");
}

// Sets the LED to an RGB color.
void setRGBLED(int r, int g, int b)
{
  ledcWrite(CHAN_0, r);
  ledcWrite(CHAN_1, g);
  ledcWrite(CHAN_2, b);
}

// Sets the LED to the next RGB color in the colorloop. TODO explore making cleaner by incrementing via HSV, and/or by adding in stable timing
void incrementRGBColorloop()
{
  switch (RGBColorloopSection) {                          // implementing the 6 sections here https://academe.co.uk/wp-content/uploads/2012/04/451px-HSV-RGB-comparison.svg_.png
    case 0:                                               // red stays up, green goes up, blue stays down
      setRGBLED(255, RGBColorloopVar, 0);
      RGBColorloopVar++;
      if (RGBColorloopVar == 255) RGBColorloopSection++;  // next section
      break;
      
    case 1:                                               // red goes down, green stays up, blue stays down
      setRGBLED(RGBColorloopVar, 255, 0);
      RGBColorloopVar--;
      if (RGBColorloopVar == 0) RGBColorloopSection++;    // next section
      break;
      
    case 2:                                               // red stays down, green stays up, blue goes up
      setRGBLED(0, 255, RGBColorloopVar);
      RGBColorloopVar++;
      if (RGBColorloopVar == 255) RGBColorloopSection++;  // next section
      break;
      
    case 3:                                               // red stays down, green goes down, blue stays up
      setRGBLED(0, RGBColorloopVar, 255);
      RGBColorloopVar--;
      if (RGBColorloopVar == 0) RGBColorloopSection++;    // next section
      break;
      
    case 4:                                               // red goes up, green stays down, blue stays up
      setRGBLED(RGBColorloopVar, 0, 255);
      RGBColorloopVar++;
      if (RGBColorloopVar == 255) RGBColorloopSection++;  // next section
      break;
      
    case 5:                                               // red stays up, green stays down, blue goes down
      setRGBLED(255, 0, RGBColorloopVar);
      RGBColorloopVar--;
      if (RGBColorloopVar == 0) RGBColorloopSection = 0;  // back to the first section
      break;
      
  }
}
