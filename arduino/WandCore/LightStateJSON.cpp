/*
  Use ESPHue to fetch desired light state, and return it in a JSON doc using ArduinoJson
  NOTE: this is custom filtered to this specific project's needs
*/

#include "LightStateJSON.h"

// Populates the given JSON document with the tailored state information for the given light.
JsonObject& getLightStateJSON(int lightID)
{
  // step 1 of 3 - fetch and prepare the raw JSON string from the Hue library
  String rawLightResponse;
  rawLightResponse = myHue.getLightInfo(lightID);
  // the next two lines trim out the non-JSON part of the response string
  int removeToIdx = rawLightResponse.indexOf("{") - 1;
  rawLightResponse.remove(0, removeToIdx);

  // step 2 of 3 - prepare a filter so we only parse the JSON fields we need
  StaticJsonDocument<112> filter;                                 // sized per recommendation from https://arduinojson.org/v6/assistant/
  JsonObject filter_state = filter.createNestedObject("state");
  filter_state["hue"] = true;
  filter_state["on"] = true;
  filter_state["effect"] = true;
  filter_state["bri"] = true;
  filter_state["sat"] = true;
  filter_state["ct"] = true;

  // step 3 of 3 - filter and parse the JSON
  StaticJsonDocument<192> doc;                                      // sized per recommendation from https://arduinojson.org/v6/assistant/
  DeserializationError error = deserializeJson(doc, rawLightResponse, DeserializationOption::Filter(filter));
  if (error) {
    debugE("deserializeJson failed: %s", error.c_str());
    // TODO -- what should happen here if there's an error?
  }

  JsonObject lightCurrentState = doc["state"];
  return lightCurrentState;
}
