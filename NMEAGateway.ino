/*

NMEA2000 Gateway.
Based on the NMEA0183ToN2k example of Timo Lappalainen.

Reads NMEA0183 messages and forwards them tho the N2k bus.
Reads compass heading and forwards it to the N2k bus.

*/

#define USE_MCP_CAN_CLOCK_SET 8
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "NMEA0183Gateway.h"
#include "CompassHeading.h"

NMEA0183Gateway * pNmea0183Gateway;
CompassHeading * pCompassHeading;

void setup() {

  Serial.begin(115200);
  setupNMEA2000Lib(&NMEA2000, &Serial);
  pNmea0183Gateway = new NMEA0183Gateway(&NMEA2000, &Serial3, &Serial,DEBUG_LEVEL_INFO);
  pCompassHeading = new CompassHeading(&NMEA2000,&Serial);
}

void loop() {

  pNmea0183Gateway->handleLoop();
  pCompassHeading->handleLoop();
}

void setupNMEA2000Lib(tNMEA2000* pNMEA2000, Stream* forwardStream) {
  
  // List here messages your device will transmit.
  unsigned long TransmitMessages[] PROGMEM={127250L,129283L,129284L,129285L,126992L,129025L,129026L,129029L,0};
  pNMEA2000->SetProductInformation("00000008",107,"NMEA0183 -> N2k -> PC","1.0.0.0 (2017-07-16)","1.0.0.0 (2017-07-16)" );
  pNMEA2000->SetDeviceInformation(8,130,25,2046);
  pNMEA2000->SetForwardStream(forwardStream);
  pNMEA2000->SetForwardType(tNMEA2000::fwdt_Text);
  pNMEA2000->SetMode(tNMEA2000::N2km_NodeOnly,25);
  pNMEA2000->EnableForward(true);
  pNMEA2000->ExtendTransmitMessages(TransmitMessages);

  
  pNMEA2000->SetForwardOwnMessages(); //Uncomment this to see the own messages we are sending.
  pNMEA2000->SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega

  pNMEA2000->Open();
}
