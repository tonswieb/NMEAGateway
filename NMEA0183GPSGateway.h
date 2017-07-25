/* 
NMEA0183GPSGateway.h

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#ifndef _NMEA0183GPSGateway_H_
#define _NMEA0183GPSGateway_H_

#include <StandardCplusplus.h>
#include <list>
#include <map>
#include <string>

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include <Time.h>
#include <MemoryFree.h>

/**
 * Defines the information needed to send a route from NMEA 0183 to NMEA 2000.
 * The last complete received route, for periodic sending.
 * A route in progress.
 * Upon receiving all WPL messages en RTE messages the routeComplete is updated and the routeInProgress cleared.
 */
struct tRoute {

  //An ordered list of waypointnames as received from RTE messages.
  std::list<std::string> wpList;
  //A map of waypoints received from WPL messages.
  std::map<std::string,tWPL> wpMap;  
  unsigned int routeID;
    
};

class NMEA0183GPSGateway {

  private:  
    unsigned long DaysSince1970;   // Days since 1970-01-01
    double Variation, Latitude, Longitude;
    //NMEA0183-BOD (Bearing Origin to Destination). Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
    tBOD bod;
    tRoute routeComplete;
    tRoute routeInProgress;
    tNMEA0183 NMEA0183_3;
    tNMEA2000* pNMEA2000;
    Stream* debugStream=0;

    void sendPGN129283(const tRMB &rmb);
    void sendPGN129284(const tRMB &rmb);
    void sendPGN129285(tRoute &route);
    void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRMB(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
    void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
    void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
    void HandleBOD(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRTE(const tNMEA0183Msg &NMEA0183Msg);
    void HandleWPL(const tNMEA0183Msg &NMEA0183Msg);
    void HandleGLL(const tNMEA0183Msg &NMEA0183Msg);
  
  public:
    NMEA0183GPSGateway(tNMEA2000* pNMEA2000, Stream* gps, Stream* debugStream = 0);
    void handleLoop();
};

#endif
