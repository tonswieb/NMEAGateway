/* 
NMEA0183GPSGateway.h

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen, Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#ifndef _NMEA0183Gateway_H_
#define _NMEA0183Gateway_H_

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include <Time.h>
#include <MemoryFree.h>

#define DEBUG_LEVEL_TRACE 4
#define DEBUG_LEVEL_DEBUG 3
#define DEBUG_LEVEL_INFO 2
#define DEBUG_LEVEL_WARN 1
#define DEBUG_LEVEL_ERROR 0

#if !defined(MAX_WP_PER_ROUTE)
  #define MAX_WP_PER_ROUTE 30
#endif


/**
 * Defines the information needed to send a route from NMEA 0183 to NMEA 2000.
 * The last complete received route, for periodic sending.
 * A route in progress.
 * Upon receiving all WPL messages en RTE messages the routeComplete is updated and the routeInProgress cleared.
 */
struct tCoordinates {
  double latitude;
  double longitude;
};

 
struct tRoute {

  //An ordered list of waypointnames as received from RTE messages.
  char names[MAX_WP_PER_ROUTE][NMEA0183_MAX_WP_NAME_LENGTH + 1];
  tCoordinates coordinates[MAX_WP_PER_ROUTE];
  /*
   * The last received RTE message set is equal to the previous received RTE message set.
   * So normally the WPL messages received in between should be aligned with the RTE message set and 
   * we should have all information to send PGN129285 messages.
   */
  boolean equalToPrevious = false;
  
  /*
   * The route is valid in case equalToPrevious is true and the waypoint names of the received WPL messages in order
   * are in sync with the order of the names received via the RTE messages.
   */
  boolean valid = false;
  //The total size of the route.
  byte size;
  /*
   * The index of the WPL messages received.
   * The first WPL after the last RTE has index 0.
   * The last WPL before the first RTE has index = size;
   */
  byte wplIndex = 0;
  /*
   * The index of the waypoint in the route which is the origin of the current leg. 
   */
  byte originCurrentLeg = 0;
  unsigned int routeID;
};

class NMEA0183Gateway {

  private:  
    unsigned long DaysSince1970;   // Days since 1970-01-01
    double Variation, Latitude, Longitude;
    //NMEA0183-BOD (Bearing Origin to Destination). Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
    tBOD bod;
    tRoute route;
    tNMEA0183 NMEA0183;
    tNMEA2000* pNMEA2000;
    Stream* debugStream=0;
    int debugLevel;

    int findOriginCurrentLeg();

    void sendPGN127250(const double &heading);
    void sendPGN129025(const double &latitude, const double &longitude);
    void sendPGN129026(const tN2kHeadingReference ref, const double &COG, const double &SOG);
    void sendPGN129029(const tGGA &gga);
    void sendPGN129283(const tRMB &rmb);
    void sendPGN129284(const tRMB &rmb);
    void sendPGN129285();

    void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRMB(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
    void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
    void HandleHDT(const tNMEA0183Msg &NMEA0183Msg);
    void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
    void HandleBOD(const tNMEA0183Msg &NMEA0183Msg);
    void HandleRTE(const tNMEA0183Msg &NMEA0183Msg);
    void HandleWPL(const tNMEA0183Msg &NMEA0183Msg);
    void HandleGLL(const tNMEA0183Msg &NMEA0183Msg);
  
  public:
    NMEA0183Gateway(tNMEA2000* pNMEA2000, Stream* nmea0183, Stream* debugStream = 0, int debugLevel = DEBUG_LEVEL_INFO);
    void handleLoop();
};

#endif
