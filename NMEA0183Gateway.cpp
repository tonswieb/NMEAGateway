/* 
NMEA0183Gateway.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen, Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include "NMEA0183Gateway.h"

#define PI_2 6.283185307179586476925286766559

//Some macros to make the logging systeem a bit less cumbersome
#define LOG_TRACE debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE
#define LOG_DEBUG debugStream!=0 && debugLevel >= DEBUG_LEVEL_DEBUG
#define LOG_INFO debugStream!=0 && debugLevel >= DEBUG_LEVEL_INFO
#define LOG_WARN debugStream!=0 && debugLevel >= DEBUG_LEVEL_WARN
#define LOG_ERROR debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR
#define log_P(...) debugStream->print(F(__VA_ARGS__))
#define logln_P(...) debugStream->println(F(__VA_ARGS__))
#define log(...) debugStream->print(__VA_ARGS__)
#define logln(...) debugStream->println(__VA_ARGS__)

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

double toMagnetic(double True, double Variation) {

  double magnetic=True-Variation;
  while (magnetic<0) magnetic+=PI_2;
  while (magnetic>=PI_2) magnetic-=PI_2;
  return magnetic;    
}

int NMEA0183Gateway::findOriginCurrentLeg() {

  for (byte i=0; i < route.size; i++) {
    if (strcmp(route.names[i], bod.originID) == 0) {
      return i;
    }
  }

  if (LOG_WARN) {
    logln_P("WARN : The origin of the leg not found in the waypoint list of the route.");    
  }
  return 0;
}

NMEA0183Gateway::NMEA0183Gateway(tNMEA2000* pNMEA2000, Stream* nmea0183, Stream* debugStream, int debugLevel) {

  this->debugStream = debugStream;
  this->debugLevel = debugLevel;
  this->pNMEA2000 = pNMEA2000;
  if (LOG_INFO) {
    log_P("INFO : Initializing NMEA0183 communication. Make sure the NMEA device uses the same baudrate ");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    log_P(" Memory free: "); logln(freeMemory());
  }
  NMEA0183.SetMessageStream(nmea0183);
  NMEA0183.Open();
}

/**
 * Resend PGN129285 at least every 5 seconds.
 * B&G Triton requires a PGN129285 message around every 10 seconds otherwise display of the destinationID is cleared.
 * Depending on how fast an updated route is constructed we are sending the same or updated route.
 */
void NMEA0183Gateway::handleLoop() {

  pNMEA2000->ParseMessages();

  tNMEA0183Msg NMEA0183Msg;  
  while (NMEA0183.GetMessage(NMEA0183Msg)) {
    HandleNMEA0183Msg(NMEA0183Msg);
  }
  
  static unsigned long timeUpdated=millis();
  if (timeUpdated+5000 < millis()) {
    timeUpdated=millis();
    sendPGN129285();
  }
}

/**
 * 129283 - Cross Track Error
 * Category: Navigation
 * This PGN provides the magnitude of position error perpendicular to the desired course.
 */
void NMEA0183Gateway::sendPGN129283(const tRMB &rmb) {

  tN2kMsg N2kMsg;
  SetN2kXTE(N2kMsg,1,N2kxtem_Autonomous, false, rmb.xte);
  pNMEA2000->SendMsg(N2kMsg);
}

/*
 * 129284 - Navigation Data
 * Category: Navigation
 * This PGN provides essential navigation data for following a route.Transmissions will originate from products that can create and manage routes using waypoints. This information is intended for navigational repeaters. 
 * 
 * Send the navigation information of the current leg of the route.
 * With long routes from NMEA0183 GPS systems it can take a while before a PGN129285 can be sent. So the display value of originID or destinationID might not always be available.
 * B&G Trition ignores OriginWaypointNumber and DestinationWaypointNumber values in this message.
 * It always takes the 2nd waypoint from PGN129285 as DestinationWaypoint.
 * Not sure if that is compliant with NMEA2000 or B&G Trition specific.
 */
void NMEA0183Gateway::sendPGN129284(const tRMB &rmb) {
  
      tN2kMsg N2kMsg;

      /*
       * PGN129285 only gives route/wp data ahead in the Active Route. So originID will always be 0 and destinationID will always be 1.
       * Unclear why these ID's need to be set in PGN129284. On B&G Triton displays other values are ignored anyway.
       */
      int originID=0;
      int destinationID=originID+1;
      
      //B&G Triton ignores the etaTime and etaDays values and does the calculation by itself. So let's leave them at 0 for now.
      double etaTime, etaDays = 0.0;
      double Mbtw = toMagnetic(rmb.btw,Variation);
      bool ArrivalCircleEntered = rmb.arrivalAlarm == 'A';
      //PerpendicularCrossed not calculated yet.
      //Need to calculate it based on current lat/long, pND->bod.magBearing and pND->rmb.lat/long
      bool PerpendicularCrossed = false;
      SetN2kNavigationInfo(N2kMsg,1,rmb.dtw,N2khr_magnetic,PerpendicularCrossed,ArrivalCircleEntered,N2kdct_RhumbLine,etaTime,etaDays,
                          bod.magBearing,Mbtw,originID,destinationID,rmb.latitude,rmb.longitude,rmb.vmg);
      pNMEA2000->SendMsg(N2kMsg);
    if (LOG_TRACE) {
      log_P("TRACE: 129284: originID="); log(bod.originID);log_P(",");logln(originID);
      log_P("TRACE: 129284: destinationID="); log(bod.destID);log_P(",");logln(destinationID);
      log_P("TRACE: 129284: latitude="); logln(rmb.latitude,5);
      log_P("TRACE: 129284: longitude="); logln(rmb.longitude,5);
      log_P("TRACE: 129284: ArrivalCircleEntered="); logln(ArrivalCircleEntered);
      log_P("TRACE: 129284: VMG="); logln(rmb.vmg);
      log_P("TRACE: 129284: DTW="); logln(rmb.dtw);
      log_P("TRACE: 129284: BTW (Current to Destination="); logln(Mbtw);
      log_P("TRACE: 129284: BTW (Orign to Desitination)="); logln(bod.magBearing);
    }
}

/**
 * 129285 - Navigation - Route/WP information
 * Category: Navigation
 * This PGN shall return Route and WP data ahead in the Active Route. It can be requested or may be transmitted without a request, typically at each Waypoint advance. 
 * 
 * Send the active route with all waypoints from the origin of the current leg and onwards.
 * So the waypoint that corresponds with the originID from the BOD message should be the 1st. The destinationID from the BOD message should be the 2nd. Etc.
 */
void NMEA0183Gateway::sendPGN129285() {

  if (!route.valid) {
    if (LOG_INFO) {
      logln_P("INFO : Skip sending PGN129285, route is not complete yet.");          
    }
    return;
  }

  tN2kMsg N2kMsg;
  SetN2kPGN129285(N2kMsg,0, 1, route.routeID, false, false, "Unknown");

  /*
   * With Garmin GPS 120 GOTO route we only get 1 waypoint. The destination. Perhaps this is the default for all NMEA0183 devices.
   * For NMEA2000 the destination needs to be 2nd waypoint in the route. So lets at the current location as the 1st waypoint in the route.
   */   
  if (route.size == 1) {
    AppendN2kPGN129285(N2kMsg, 0, "CURRENT", Latitude, Longitude);
    AppendN2kPGN129285(N2kMsg, 1, route.names[0], route.coordinates[0].latitude, route.coordinates[0].longitude);
    if (LOG_TRACE) {
        log_P("TRACE: 129285:");log("CURRENT");log_P(",");log(Latitude);log_P(",");log(Longitude);log_P("\n");      
        log_P("TRACE: 129285:");log(route.names[0]);log_P(",");log(route.coordinates[0].latitude);log_P(",");log(route.coordinates[0].longitude);log_P("\n");      
    }
  } else {
    for (byte i=route.originCurrentLeg; i < route.size; i++) {
      byte j = i - route.originCurrentLeg;
      //Continue adding waypoints as long as they fit within a single message
      if (LOG_TRACE) {
        log_P("TRACE: 129285:");log(route.names[i]);log_P(",");log(route.coordinates[i].latitude);log_P(",");log(route.coordinates[i].longitude);log_P("\n");
      }
      if (!AppendN2kPGN129285(N2kMsg, j, route.names[i], route.coordinates[i].latitude, route.coordinates[i].longitude)) {
        //Max. nr. of waypoints per message is reached.Send a message with all waypoints upto this one and start constructing a new message.
        pNMEA2000->SendMsg(N2kMsg); 
        N2kMsg.Clear();
        SetN2kPGN129285(N2kMsg,j, 1, route.routeID, false, false, "Unknown");
        //TODO: Check for the result of the Append, should not fail due to message size. Perhaps some other reason?
        AppendN2kPGN129285(N2kMsg, j, route.names[i], route.coordinates[i].latitude, route.coordinates[i].longitude);
      }
    }
  }
  pNMEA2000->SendMsg(N2kMsg);       
}

void NMEA0183Gateway::sendPGN129029(const tGGA &gga) {
  
    tN2kMsg N2kMsg;
    SetN2kGNSS(N2kMsg,1,DaysSince1970,gga.GPSTime,gga.latitude,gga.longitude,gga.altitude,
              N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(gga.GPSQualityIndicator),gga.satelliteCount,gga.HDOP,0,
              gga.geoidalSeparation,1,N2kGNSSt_GPS,gga.DGPSReferenceStationID,gga.DGPSAge
              );
    pNMEA2000->SendMsg(N2kMsg); 
}

void NMEA0183Gateway::sendPGN129025(const double &latitude, const double &longitude) {

    tN2kMsg N2kMsg;
    SetN2kLatLonRapid(N2kMsg, latitude, longitude);
    pNMEA2000->SendMsg(N2kMsg);
}

void NMEA0183Gateway::sendPGN129026(const tN2kHeadingReference ref, const double &COG, const double &SOG) {

    tN2kMsg N2kMsg;
    SetN2kCOGSOGRapid(N2kMsg,1,ref,COG,SOG);
    pNMEA2000->SendMsg(N2kMsg);
}

void NMEA0183Gateway::sendPGN127250(const double &trueHeading) {

    tN2kMsg N2kMsg;
    double MHeading = toMagnetic(trueHeading,Variation);
    SetN2kMagneticHeading(N2kMsg,1,MHeading,0,Variation);
    pNMEA2000->SendMsg(N2kMsg);
}

// NMEA0183 message Handler functions
void NMEA0183Gateway::HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {

  tRMC rmc;
  if (NMEA0183ParseRMC(NMEA0183Msg,rmc) && rmc.status == 'A') {
    tN2kMsg N2kMsg;
    double MCOG = toMagnetic(rmc.trueCOG,rmc.variation);
    sendPGN129026(N2khr_magnetic,MCOG,rmc.SOG);
    sendPGN129025(rmc.latitude,rmc.longitude);
    Latitude = rmc.latitude;
    Longitude = rmc.longitude;
    DaysSince1970 = rmc.daysSince1970;
    Variation = rmc.variation;
    if (LOG_TRACE) {
      log_P("TRACE: RMC: GPSTime="); logln(rmc.GPSTime);
      log_P("TRACE: RMC: Latitude="); logln(rmc.latitude,5);
      log_P("TRACE: RMC: Longitude="); logln(rmc.longitude,5);
      log_P("TRACE: RMC: COG="); logln(rmc.trueCOG);
      log_P("TRACE: RMC: SOG="); logln(rmc.SOG);
      log_P("TRACE: RMC: DaysSince1970="); logln(rmc.daysSince1970);
      log_P("TRACE: RMC: Variation="); logln(rmc.variation);
    }
  } else if (rmc.status == 'V' && LOG_WARN) { logln_P("WARN : RMC is Void");
  } else if (LOG_ERROR) { logln_P("ERROR: Failed to parse RMC"); }
}

/**
 * Receive NMEA0183 RMB message (Recommended Navigation Data for GPS).
 * Contains enough information to send a NMEA2000 PGN129283 (XTE) message and NMEA2000 PGN129284 message.
 */
void NMEA0183Gateway::HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {

  tRMB rmb;
  if (NMEA0183ParseRMB(NMEA0183Msg, rmb)  && rmb.status == 'A') {
    sendPGN129283(rmb);
    sendPGN129284(rmb);
    if (LOG_TRACE) {
      log_P("TRACE: RMB: XTE="); logln(rmb.xte);
      log_P("TRACE: RMB: DTW="); logln(rmb.dtw);
      log_P("TRACE: RMB: BTW="); logln(rmb.btw);
      log_P("TRACE: RMB: VMG="); logln(rmb.vmg);
      log_P("TRACE: RMB: OriginID="); logln(rmb.originID);
      log_P("TRACE: RMB: DestinationID="); logln(rmb.destID);
      log_P("TRACE: RMB: Latittude="); logln(rmb.latitude,5);
      log_P("TRACE: RMB: Longitude="); logln(rmb.longitude,5);
    }
  } else if (rmb.status == 'V' && LOG_WARN) { logln_P("WARN : RMB is Void");
  } else if (LOG_ERROR) { logln_P("ERROR: Failed to parse RMB"); }
}

void NMEA0183Gateway::HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {

  tGGA gga;
  if (NMEA0183ParseGGA(NMEA0183Msg,gga) && gga.GPSQualityIndicator > 0) {
    sendPGN129029(gga);
    Latitude = gga.latitude;
    Longitude = gga.longitude;

    if (LOG_TRACE) {
      log_P("TRACE: GGA: Time="); logln(gga.GPSTime);
      log_P("TRACE: GGA: Latitude="); logln(gga.latitude,5);
      log_P("TRACE: GGA: Longitude="); logln(gga.longitude,5);
      log_P("TRACE: GGA: Altitude="); logln(gga.altitude,1);
      log_P("TRACE: GGA: GPSQualityIndicator="); logln(gga.GPSQualityIndicator);
      log_P("TRACE: GGA: SatelliteCount="); logln(gga.satelliteCount);
      log_P("TRACE: GGA: HDOP="); logln(gga.HDOP);
      log_P("TRACE: GGA: GeoidalSeparation="); logln(gga.geoidalSeparation);
      log_P("TRACE: GGA: DGPSAge="); logln(gga.DGPSAge);
      log_P("TRACE: GGA: DGPSReferenceStationID="); logln(gga.DGPSReferenceStationID);
    }
  } else if (gga.GPSQualityIndicator == 0 && LOG_WARN) { logln_P("WARN : GGA invalid GPS fix.");
  } else if (LOG_ERROR) { logln_P("ERROR: Failed to parse GGA"); }
}

void NMEA0183Gateway::HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {

  tGLL gll;
  if (NMEA0183ParseGLL(NMEA0183Msg,gll) && gll.status == 'A') {
    sendPGN129025(gll.latitude,gll.longitude);
    Latitude = gll.latitude;
    Longitude = gll.longitude;

    if (LOG_TRACE) {
      log_P("TRACE: GLL: Time="); logln(gll.GPSTime);
      log_P("TRACE: GLL: Latitude="); logln(gll.latitude,5);
      log_P("TRACE: GLL: Longitude="); logln(gll.longitude,5);
    }
  } else if (gll.status == 'V' && LOG_WARN) { logln_P("WARN : GLL is Void");
  } else if (LOG_ERROR) { logln_P("ERROR: Failed to parse GLL"); }
}

void NMEA0183Gateway::HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {

  double TrueHeading;
  if (NMEA0183ParseHDT(NMEA0183Msg,TrueHeading)) {
    sendPGN127250(TrueHeading);
    if (LOG_TRACE) {
      log_P("TRACE: True heading="); logln(TrueHeading);
    }
  } else if (LOG_ERROR) { logln_P("Failed to parse HDT"); }
}

void NMEA0183Gateway::HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG, COG, SOG;
  
  if (NMEA0183ParseVTG(NMEA0183Msg,COG,MagneticCOG,SOG)) {
    Variation=COG-MagneticCOG; // Save variation for Magnetic heading
    sendPGN129026(N2khr_true,COG,SOG);
    if (LOG_TRACE) {
      log_P("TRACE: COG="); logln(COG);
    }
  } else if (LOG_ERROR) { logln_P("Failed to parse VTG"); }
}

/**
 * Receive NMEA0183 BOD message (Bearing Origin to Destination) and store it for later use when the RMB message is received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void NMEA0183Gateway::HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183ParseBOD(NMEA0183Msg,bod)) {
    if (LOG_TRACE) {
      log_P("TRACE: BOD: True heading="); logln(bod.trueBearing);
      log_P("TRACE: BOD: Magnetic heading="); logln(bod.magBearing);
      log_P("TRACE: BOD: Origin ID="); logln(bod.originID);
      log_P("TRACE: BOD: Dest ID="); logln(bod.destID);
    }
  } else if (LOG_ERROR) { logln_P("Failed to parse BOD"); }
}

/**
 * Handle receiving NMEA0183 RTE message (route).
 * Based on the info which is processed after the last correlated RTE message is received a NMEA2000 messages is periodically sent using a timer.
 * A timer is used because (at least) B&G Triton requires a new PGN129285 around every 10 sec, but receiving RTE messages could take much longer 
 * on routes with more then 5 waypoints. So we need to sent the same PGN129285 multiple times in between.
 * For this we also need previously send WPL, messages. A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) followed 
 * by the RTE message(s) in the next cycle. So it can take a while for long routes before alle waypoints are received and finally the RTE messages.
 */
void NMEA0183Gateway::HandleRTE(const tNMEA0183Msg &NMEA0183Msg) {

  tRTE rte;
  if (NMEA0183ParseRTE(NMEA0183Msg,rte)) {

    if (rte.currSentence == 1) {
      //Assume the route received is equal to the previous route received until proven otherwise based on
      //comparing the waypoints names received and hence assume all coordinates are received as well 
      //in between the two received routes.
      route.equalToPrevious = true;
      route.valid = false;
      route.routeID = rte.routeID;
      route.size = 0;
      route.wplIndex = 0;
    }

    //Combine the waypoints of correlated RTE messages in a central list.
    //Will be processed when the last RTE message is recevied.
    for (int i=0; i<rte.nrOfwp; i++) {
      if (route.size >= MAX_WP_PER_ROUTE) {
        if (LOG_WARN) {
          log_P("WARN : Maximum waypoints per route is reached. Ignoring waypoint: ");logln(rte[i]);
        }
      }
      else if (route.equalToPrevious == false || strcmp(route.names[route.size], rte[i]) != 0) {
        strcpy(route.names[route.size], rte[i]);
        route.equalToPrevious = false;
      }
      route.size++;
    }
    if (LOG_TRACE) {
      log_P("TRACE: RTE equal to previous: ");logln(route.equalToPrevious);
      log_P("TRACE: Waypoints in RTE message: ");
      for (byte i=0; i<rte.nrOfwp;i++) {
        log(rte[i]);log_P(",");
      }
      log("\n");      
      log_P("TRACE: Waypoints in Route list: ");
      for (byte i=0; i<route.size;i++) {
        log(route.names[i]);log_P(",");
      }
      log("\n");
    }

    if (rte.currSentence == rte.nrOfsentences) {
      if (route.size != 1 && rte.type == 'c') {
        route.originCurrentLeg = findOriginCurrentLeg();
      } else {
        //No need to find origin when rte.type == 'w', because the 1st wp is the origin
        route.originCurrentLeg = 0;
      }
      if (route.equalToPrevious) {
      	//The RTE is marked as valid after receiving the same RTE sequence twice
      	//and alle WPL messages in between with the coordinates.
        route.valid = true;
      }
      //Sending PGN129285 is handled from a timer.
    }

    if (LOG_TRACE) {
      log_P("TRACE: RTE: Time="); logln(millis());
      log_P("TRACE: RTE: Nr of sentences="); logln(rte.nrOfsentences);
      log_P("TRACE: RTE: Current sentence="); logln(rte.currSentence);
      log_P("TRACE: RTE: Type="); logln(rte.type);
      log_P("TRACE: RTE: Route ID="); logln(rte.routeID);
    }
  } else if (LOG_ERROR) { logln_P("Failed to parse RTE"); }
}

/**
 * Receive NMEA0183 WPL message (Waypoint List) and store it for later use after all RTE messages are received.
 * A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) so it can take a while for long routes 
 * before alle waypoints are received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void NMEA0183Gateway::HandleWPL(const tNMEA0183Msg &NMEA0183Msg) {
  
  tWPL wpl;
  if (NMEA0183ParseWPL(NMEA0183Msg,wpl)) {
    byte i = route.wplIndex;
    if (route.wplIndex >= MAX_WP_PER_ROUTE) {
        if (LOG_WARN) {
          log_P("WARN : Maximum waypoints per route is reached. Ignoring waypoint: ");logln(wpl.name);
        }      
    } else if (strcmp(route.names[i],wpl.name) == 0) {
      tCoordinates coordinates;
      coordinates.latitude = wpl.latitude;
      coordinates.longitude = wpl.longitude;
      route.coordinates[i] = coordinates;
      if (LOG_TRACE) {
        log_P("TRACE: WPL: Time="); logln(millis());
        log_P("TRACE: WPL: Name="); logln(route.names[i]);
        log_P("TRACE: WPL: Latitude="); logln(route.coordinates[i].latitude);
        log_P("TRACE: WPL: Longitude="); logln(route.coordinates[i].longitude);
      }      
      route.wplIndex++;     
    } else {
      //Invalidate the route. Apparently the RTE messages are not in sync with the WPL messages.
      //Normally WPL messages are send in the same order as the order in the RTE messages.
      //Should be an exceptional case, for example when we cannot keep up. So let's wait for a new RTE message and try again.
      route.valid = false;
      if (LOG_WARN) {
        logln_P("WARN : The received WPL message is not in sync with the previously received RTE messages.");
      }
      if (LOG_DEBUG) {
        log_P("DEBUG: RTE: Name="); logln(route.names[i]);
        log_P("DEBUG: WPL: Name="); logln(wpl.name);
        log_P("DEBUG: RTL: Index="); logln(i);
      }      
    }
  } else if (LOG_ERROR) { logln_P("Failed to parse WPL"); }
}

/*
 * NMEA0183Msg.IsMessageCode wrapper function using PROGMEM Strings to limited SRAM usage.
 */
boolean isMessageCode_P(const tNMEA0183Msg &NMEA0183Msg, const char* code) {
  
  char buffer[4];
  strcpy_P(buffer, code); //Copy from PROGMEM to SRAM
  return NMEA0183Msg.IsMessageCode(buffer);
}

void NMEA0183Gateway::HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {

  if (LOG_DEBUG) {
      log_P("DEBUG : Memory free: "); log(freeMemory());
      log_P(" Handling NMEA0183 message "); logln(NMEA0183Msg.MessageCode());
  }

  if (isMessageCode_P(NMEA0183Msg,PSTR("GGA"))) {
    HandleGGA(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("GGL"))) {
    HandleGLL(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("RMB"))) {
    HandleRMB(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("RMC"))) {
    HandleRMC(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("WPL"))) {
    HandleWPL(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("BOD"))) {
    HandleBOD(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("VTG"))) {
    HandleVTG(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("HDT"))) {
    HandleHDT(NMEA0183Msg);
  } else if (isMessageCode_P(NMEA0183Msg,PSTR("RTE"))) {
    HandleRTE(NMEA0183Msg);
  }
} 
