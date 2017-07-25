/* 
CompassHeading.h

Author: Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#ifndef _CompassHeading_H_
#define _CompassHeading_H_

#include <Wire.h>
#include <HMC5883L.h>

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <Time.h>

class CompassHeading {

  private:
	HMC5883L compass;
    Stream* debugStream=0;
	tNMEA2000* pNMEA2000;

	void sendPGN127250(const double &magHeading, const double &variation, const double &deviation);
  
  public:
    CompassHeading(tNMEA2000* pNMEA2000, Stream* debugStream = 0);
    void handleLoop();
};

#endif
