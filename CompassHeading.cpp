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

  CompassHeading::CompassHeading(tNMEA2000* pNMEA2000, Stream* debugStream = 0, unsigned long updateInterval) {

  this->debugStream = debugStream;
  this->pNMEA2000 = pNMEA2000;
  this->updateInterval = updateInterval;

  // Initialize Initialize HMC5883L
	  while (!compass.begin())
	  {
		  if (debugStream != 0) {
	  	    debugStream->println("Could not find a valid HMC5883L sensor, check wiring!");		  	
		  }
	    delay(500);
	  }  	

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

double scaleCorrection(double heading) {
  
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  return heading;
}

void CompassHeading::handleLoop() {

  if (lastUpdate+updateInterval < millis()) {
    lastUpdate=millis();
    Vector norm = compass.readNormalize();

    // Calculate heading
    double heading = atan2(norm.YAxis, norm.XAxis);
    heading = scaleCorrection(heading);
  	//TODO: set deviation and variation
  	//TODO: Check if heading is in radians
  	sendPGN127250(heading,N2kDoubleNA,N2kDoubleNA);
  }
}

void CompassHeading::sendPGN127250(const double &magHeading, const double &variation, const double &deviation) {

    tN2kMsg N2kMsg;
    SetN2kMagneticHeading(N2kMsg,1,magHeading,deviation, variation);
    pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0) {
      debugStream->print("Compass: Heading="); debugStream->println(magHeading);
      debugStream->print("Compass: Deviation="); debugStream->println(deviation);
      debugStream->print("Compass: Variation="); debugStream->println(variation);
    }
}
