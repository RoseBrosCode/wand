#ifndef GLOBALS
#define GLOBALS
////////////////////////////////////////////////////////////////////////////////////
// local environment
////////////////////////////////////////////////////////////////////////////////////
#include "environ.h"
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// WiFi Client
////////////////////////////////////////////////////////////////////////////////////
#include <WiFiClient.h>
#include <WiFi.h>

// Connects to WiFi
extern WiFiClient client;
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// mDNS
////////////////////////////////////////////////////////////////////////////////////
#include <DNSServer.h>
#include <ESPmDNS.h>
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Set up remote debug
////////////////////////////////////////////////////////////////////////////////////
#include <RemoteDebug.h>        // https://github.com/JoaoLopesF/RemoteDebug

extern RemoteDebug Debug;       // from https://github.com/JoaoLopesF/RemoteDebug/issues/65
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Hue Client
////////////////////////////////////////////////////////////////////////////////////
// Add Hue lib
#include <ESPHue.h>

// Hue lib singleton
extern ESPHue myHue = ESPHue(client, myHUEAPIKEY, myHUEBRIDGEIP, 80);
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// JSON Library
////////////////////////////////////////////////////////////////////////////////////
#include <ArduinoJson.h>
////////////////////////////////////////////////////////////////////////////////////

#endif
