/*
 Demo: NMEA0183 library. NMEA0183 -> NMEA2000
   Reads messages from NMEA0183 and forwards them to the N2k bus
   Also forwards all NMEA2000 bus messages to the PC (Serial)

 This example reads NMEA0183 messages from one serial port. It is possible
 to add more serial ports for having NMEA0183 combiner functionality.

 The messages, which will be handled has been defined on NMEA0183Handlers.cpp
 on NMEA0183Handlers variable initialization. So this does not automatically
 handle all NMEA0183 messages. If there is no handler for some message you need,
 you have to write handler for it and add it to the NMEA0183Handlers variable
 initialization. If you write new handlers, please after testing send them to me,
 so I can add them for others use.
*/

#include "NMEA0183Handlers.h"

NMEA0183Handler * phandler;

void setup() {

  Serial.begin(115200);
  phandler = new NMEA0183Handler(&Serial3, &Serial);
//  phandler = new NMEA0183Handler(&Serial3, &Serial, &Serial, false);
}

void loop() {

  phandler->handleLoop();
}


