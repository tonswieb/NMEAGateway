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

//#define N2K_SOURCE 15
//#define USE_N2K_MCP_CAN USE_N2K_MCP_CAN
//#define N2k_CAN_INT_PIN 10
//#define N2k_SPI_CS_PIN 53
#define USE_MCP_CAN_CLOCK_SET 8

#include <StandardCplusplus.h>
#include <Arduino.h>
#include <Time.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include "BoatData.h"

#define NMEA0183SourceGPSCompass 3
#define NMEA0183SourceGPS 1

tBoatData BoatData;
tNavData NavData;

tNMEA0183Msg NMEA0183Msg;
tNMEA0183 NMEA0183_3;

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={129283L,129284L,126992L,129026L,129029L,0};

void setup() {

  // Setup NMEA2000 system
  Serial.begin(115200);
  NMEA2000.SetProductInformation("00000008", // Manufacturer's Model serial code
                                 107, // Manufacturer's product code
                                 "NMEA0183 -> N2k -> PC",  // Manufacturer's Model ID
                                 "1.0.0.1 (2015-11-18)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2015-11-18)" // Manufacturer's Model version
                                 );
  // Det device information
  NMEA2000.SetDeviceInformation(8, // Unique number. Use e.g. Serial number.
                                130, // Device function=PC Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );


  NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,25);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  // Setup NMEA0183 ports and handlers
  InitNMEA0183Handlers(&NMEA2000, &BoatData, &NavData);
  DebugNMEA0183Handlers(&Serial);
  NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);

  NMEA0183_3.Begin(&Serial3,NMEA0183SourceGPSCompass, 4800);
}

void loop() {

  NMEA2000.ParseMessages();
  NMEA0183_3.ParseMessages();
}

