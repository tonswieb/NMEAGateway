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

#define USE_MCP_CAN_CLOCK_SET 8
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "NMEA0183GPSGateway.h"
#include "CompassHeading.h"

NMEA0183GPSGateway * pGpsGateway;
CompassHeading * pCompassHeading;
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127250L,129283L,129284L,129285L,126992L,129025L,129026L,129029L,0};

void setup() {

  Serial.begin(115200);
  setupNMEA2000Lib(&NMEA2000, &Serial);
  pGpsGateway = new NMEA0183GPSGateway(&NMEA2000, &Serial3, &Serial);
//  pCompassHeading = new CompassHeading(&NMEA2000,&Serial);
}

void loop() {

  pGpsGateway->handleLoop();
//  pCompassHeading->handleLoop();
}

void setupNMEA2000Lib(tNMEA2000* pNMEA2000, Stream* forwardStream) {
  
  pNMEA2000->SetProductInformation("00000008",107,"NMEA0183 -> N2k -> PC","1.0.0.0 (2017-07-16)","1.0.0.0 (2017-07-16)" );
  pNMEA2000->SetDeviceInformation(8,130,25,2046);
  pNMEA2000->SetForwardStream(forwardStream);
  pNMEA2000->SetForwardType(tNMEA2000::fwdt_Text);
  pNMEA2000->SetMode(tNMEA2000::N2km_NodeOnly,25);
  pNMEA2000->EnableForward(true);
  pNMEA2000->ExtendTransmitMessages(TransmitMessages);
  pNMEA2000->Open();
}
