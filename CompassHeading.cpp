/* 
CompassHeading.cpp

Author: Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include "CompassHeading.h"

  CompassHeading::CompassHeading(tNMEA2000* pNMEA2000, Stream* debugStream = 0) {

  this->debugStream = debugStream;
  this->pNMEA2000 = pNMEA2000;

  // Initialize Initialize HMC5883L
	  while (!compass.begin())
	  {
		  if (debugStream != 0) {
	  	    debugStream->println("Could not find a valid HMC5883L sensor, check wiring!");		  	
		  }
	    delay(500);
	  }  	

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

void CompassHeading::handleLoop() {

  static unsigned long timeUpdated=millis();
  if (timeUpdated+5000 < millis()) {
    timeUpdated=millis();
    Vector norm = compass.readNormalize();

    // Calculate heading
    double heading = atan2(norm.YAxis, norm.XAxis);
  	//TODO: set deviation and variation
  	//TODO: Check if heading is in radians
  	sendPGN127250(heading,N2kDoubleNA,N2kDoubleNA);
  }
}

void CompassHeading::sendPGN127250(const double &magHeading, const double &variation, const double &deviation) {

    tN2kMsg N2kMsg;
    SetN2kMagneticHeading(N2kMsg,1,magHeading);
    pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0) {
      debugStream->print("Compass: Heading="); debugStream->println(magHeading);
      debugStream->print("Compass: Deviation="); debugStream->println(deviation);
      debugStream->print("Compass: Variation="); debugStream->println(variation);
    }
}
