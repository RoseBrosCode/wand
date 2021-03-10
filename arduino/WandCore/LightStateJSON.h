/*
  Use ESPHue to fetch desired light state, and return it in a JSON doc using ArduinoJson
  NOTE: this is custom filtered to this specific project's needs
*/

#ifndef LIGHT_STATE_JSON
#define LIGHT_STATE_JSON

#include "Globals.h"

// Populates the given JSON document with the tailored state information for the given light.
JsonObject& getLightStateJSON(int lightID);

#endif
