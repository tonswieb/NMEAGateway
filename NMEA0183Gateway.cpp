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

/**
 * With Garmin GPS 120 GOTO route we only get 1 waypoint. The destination.
 * Perhaps this is the default for all NMEA0183 devices.
 * For NMEA2000 the destination needs to be 2nd waypoint in the route. 
 * So lets at the current location as the 1st waypoint in the route.
 */
void handleGarminGPS120GOTO(tRoute &route, double latitude, double longitude) {
  
  tWPL wpl;
  wpl.name = "CURRENT";
  wpl.latitude = latitude;
  wpl.longitude = longitude;
  route.wpMap[wpl.name] = wpl;
  route.wpList.insert(route.wpList.begin(),wpl.name);
}

void NMEA0183Gateway::removeWaypointsUpToOriginCurrentLeg(tRoute &route, const std::string originID) {

  bool originFound=false;
  std::list<std::string>::iterator it = route.wpList.begin();
  for (; it!=route.wpList.end(); ++it) {
    if (*it == originID) {
      originFound=true;
      break;
    }
  }
  if (originFound) {
    route.wpList.erase(route.wpList.begin(),it);
  } else if (debugStream !=0 && debugLevel >= DEBUG_LEVEL_WARN) {
    //Should normally not occur the BOD and RTE's are send in the same NMEA0183 cycle so thesee should be in sync.
    debugStream->println(F("WARN : The origin of the leg not found in the waypoint list of the route."));
  }
}

/*
 * Very inefficient method, perhaps calling this method should be preceded by calculating a hash over the route to find out if the route has changed.
 * Another alternative could be creating a temporary map, by moving (swapping) the entries still needed, clearing the original map and swapping the temporary items back.
 * If we swap item by item it should not have much memory overhead.
 */
void NMEA0183Gateway::removeUnusedWaypointsFromRoute(tRoute &route) {

  //TODO: See how we can reclaim memory in case the route is shrinked to a shorter one.
  for (std::map<std::string,tWPL>::iterator it = route.wpMap.begin(); it!=route.wpMap.end(); ++it) {
    boolean mapElementFoundInWpList = false;
    for (std::string currWp : route.wpList) {
      if (it->first == currWp) {
        mapElementFoundInWpList = true;
        break;
      }
    }
    if (!mapElementFoundInWpList) {
      if (debugStream !=0 && debugLevel >= DEBUG_LEVEL_INFO) {
        debugStream->print(F("INFO : Removing WPL which is no longer in route: "));debugStream->println(it->first.c_str());
      }
      route.wpMap.erase(it);
    }
  }
}




NMEA0183Gateway::NMEA0183Gateway(tNMEA2000* pNMEA2000, Stream* nmea0183, Stream* debugStream, int debugLevel) {

  this->debugStream = debugStream;
  this->debugLevel = debugLevel;
  this->pNMEA2000 = pNMEA2000;
  memoryMin = freeMemory();
  if (debugStream !=0 && debugLevel >= DEBUG_LEVEL_INFO) {
    debugStream->print(F("INFO : Initializing NMEA0183 communication at "));debugStream->print(9600);debugStream->println(F(" baud. Make sure the NMEA device uses the same baudrate."));
    debugStream->print(F(" Memory Min: ")); debugStream->println(memoryMin);
  }
  NMEA0183.Begin(nmea0183,3, 9600);
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
    if (freeMemory() < memoryMin) {
        memoryMin = freeMemory();
    }
  }
  
  static unsigned long timeUpdated=millis();
  if (timeUpdated+5000 < millis()) {
    timeUpdated=millis();
    if (!route.wpListInProgress) {
      sendPGN129285(route);    
    } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_INFO) {
      debugStream->println(F("INFO : Skip sending PGN129285, because waypoint list for route is being rebuild."));
    }
    if (freeMemory() < memoryMin) {
        memoryMin = freeMemory();
    }
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
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: 129284: originID=")); debugStream->print(bod.originID.c_str());debugStream->print(F(","));debugStream->println(originID);
      debugStream->print(F("TRACE: 129284: destinationID=")); debugStream->print(bod.destID.c_str());debugStream->print(F(","));debugStream->println(destinationID);
      debugStream->print(F("TRACE: 129284: latitude=")); debugStream->println(rmb.latitude,5);
      debugStream->print(F("TRACE: 129284: longitude=")); debugStream->println(rmb.longitude,5);
      debugStream->print(F("TRACE: 129284: ArrivalCircleEntered=")); debugStream->println(ArrivalCircleEntered);
      debugStream->print(F("TRACE: 129284: VMG=")); debugStream->println(rmb.vmg);
      debugStream->print(F("TRACE: 129284: DTW=")); debugStream->println(rmb.dtw);
      debugStream->print(F("TRACE: 129284: BTW (Current to Destination=")); debugStream->println(Mbtw);
      debugStream->print(F("TRACE: 129284: BTW (Orign to Desitination)=")); debugStream->println(bod.magBearing);
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
void NMEA0183Gateway::sendPGN129285(tRoute &route) {

  //TODO: Until the first complete cycle of alle WPL and RTE messages, the PGN send is rubish and should be avoided.
  tN2kMsg N2kMsg;
  int i=0;
  SetN2kPGN129285(N2kMsg,i, 1, route.routeID, false, false, "Unknown");

  for (std::string currWp : route.wpList) {
    //Continue adding waypoints as long as they fit within a single message
    tWPL wpl = route.wpMap[currWp];
    if (wpl.name == currWp) {
      if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
        debugStream->print(F("TRACE: 129285:"));
        debugStream->print(wpl.name.c_str());debugStream->print(F(","));
        debugStream->print(wpl.latitude);debugStream->print(F(","));
        debugStream->print(wpl.longitude);debugStream->print(F("\n"));
      }
      if (!AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude)) {
        //Max. nr. of waypoints per message is reached.Send a message with all waypoints upto this one and start constructing a new message.
        pNMEA2000->SendMsg(N2kMsg); 
        N2kMsg.Clear();
        SetN2kPGN129285(N2kMsg,i, 1, route.routeID, false, false, "Unknown");
        //TODO: Check for the result of the Append, should not fail due to message size. Perhaps some other reason?
        AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude);
      }
    } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE)  {
        debugStream->print(F("TRACE: 129285: Skipping "));
        debugStream->print(currWp.c_str());debugStream->print(F("\n"));
    }
    i++;
  }
  pNMEA2000->SendMsg(N2kMsg);       
}

// NMEA0183 message Handler functions
void NMEA0183Gateway::HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {

  tRMC rmc;
  if (NMEA0183ParseRMC(NMEA0183Msg,rmc) && rmc.status == 'A') {
    tN2kMsg N2kMsg;
    double MCOG = toMagnetic(rmc.trueCOG,rmc.variation);
    //PGN129026
    SetN2kCOGSOGRapid(N2kMsg,1,N2khr_magnetic,MCOG,rmc.SOG); 
    pNMEA2000->SendMsg(N2kMsg);
    //PGN129025
    SetN2kLatLonRapid(N2kMsg,rmc.latitude,rmc.longitude);
    pNMEA2000->SendMsg(N2kMsg);
    Latitude = rmc.latitude;
    Longitude = rmc.longitude;
    DaysSince1970 = rmc.daysSince1970;
    Variation = rmc.variation;
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: RMC: GPSTime=")); debugStream->println(rmc.GPSTime);
      debugStream->print(F("TRACE: RMC: Latitude=")); debugStream->println(rmc.latitude,5);
      debugStream->print(F("TRACE: RMC: Longitude=")); debugStream->println(rmc.longitude,5);
      debugStream->print(F("TRACE: RMC: COG=")); debugStream->println(rmc.trueCOG);
      debugStream->print(F("TRACE: RMC: SOG=")); debugStream->println(rmc.SOG);
      debugStream->print(F("TRACE: RMC: DaysSince1970=")); debugStream->println(rmc.daysSince1970);
      debugStream->print(F("TRACE: RMC: Variation=")); debugStream->println(rmc.variation);
    }
  } else if (debugStream!=0 && rmc.status == 'V' && debugLevel >= DEBUG_LEVEL_WARN) { debugStream->println(F("WARN : RMC is Void"));
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR ) { debugStream->println(F("ERROR: Failed to parse RMC")); }
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
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: RMB: XTE=")); debugStream->println(rmb.xte);
      debugStream->print(F("TRACE: RMB: DTW=")); debugStream->println(rmb.dtw);
      debugStream->print(F("TRACE: RMB: BTW=")); debugStream->println(rmb.btw);
      debugStream->print(F("TRACE: RMB: VMG=")); debugStream->println(rmb.vmg);
      debugStream->print(F("TRACE: RMB: OriginID=")); debugStream->println(rmb.originID.c_str());
      debugStream->print(F("TRACE: RMB: DestinationID=")); debugStream->println(rmb.destID.c_str());
      debugStream->print(F("TRACE: RMB: Latittude=")); debugStream->println(rmb.latitude,5);
      debugStream->print(F("TRACE: RMB: Longitude=")); debugStream->println(rmb.longitude,5);
    }
  } else if (debugStream!=0 && rmb.status == 'V' && debugLevel >= DEBUG_LEVEL_WARN) { debugStream->println(F("WARN : RMB is Void"));
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("ERROR: Failed to parse RMB")); }
}

void NMEA0183Gateway::HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {

  tGGA gga;
  if (NMEA0183ParseGGA(NMEA0183Msg,gga) && gga.GPSQualityIndicator > 0) {
    tN2kMsg N2kMsg;
    //129029
    SetN2kGNSS(N2kMsg,1,DaysSince1970,gga.GPSTime,gga.latitude,gga.longitude,gga.altitude,
              N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(gga.GPSQualityIndicator),gga.satelliteCount,gga.HDOP,0,
              gga.geoidalSeparation,1,N2kGNSSt_GPS,gga.DGPSReferenceStationID,gga.DGPSAge
              );
    pNMEA2000->SendMsg(N2kMsg); 
    Latitude = gga.latitude;
    Longitude = gga.longitude;

    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: GGA: Time=")); debugStream->println(gga.GPSTime);
      debugStream->print(F("TRACE: GGA: Latitude=")); debugStream->println(gga.latitude,5);
      debugStream->print(F("TRACE: GGA: Longitude=")); debugStream->println(gga.longitude,5);
      debugStream->print(F("TRACE: GGA: Altitude=")); debugStream->println(gga.altitude,1);
      debugStream->print(F("TRACE: GGA: GPSQualityIndicator=")); debugStream->println(gga.GPSQualityIndicator);
      debugStream->print(F("TRACE: GGA: SatelliteCount=")); debugStream->println(gga.satelliteCount);
      debugStream->print(F("TRACE: GGA: HDOP=")); debugStream->println(gga.HDOP);
      debugStream->print(F("TRACE: GGA: GeoidalSeparation=")); debugStream->println(gga.geoidalSeparation);
      debugStream->print(F("TRACE: GGA: DGPSAge=")); debugStream->println(gga.DGPSAge);
      debugStream->print(F("TRACE: GGA: DGPSReferenceStationID=")); debugStream->println(gga.DGPSReferenceStationID);
    }
  } else if (debugStream!=0 && gga.GPSQualityIndicator == 0 && debugLevel >= DEBUG_LEVEL_WARN) { debugStream->println(F("WARN : GGA invalid GPS fix."));
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("ERROR: Failed to parse GGA")); }
}

void NMEA0183Gateway::HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {

  tGLL gll;
  if (NMEA0183ParseGLL(NMEA0183Msg,gll) && gll.status == 'A') {
    tN2kMsg N2kMsg;
    //PGN129025
    SetN2kLatLonRapid(N2kMsg, gll.latitude, gll.longitude);
    pNMEA2000->SendMsg(N2kMsg);
    Latitude = gll.latitude;
    Longitude = gll.longitude;

    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: GLL: Time=")); debugStream->println(gll.GPSTime);
      debugStream->print(F("TRACE: GLL: Latitude=")); debugStream->println(gll.latitude,5);
      debugStream->print(F("TRACE: GLL: Longitude=")); debugStream->println(gll.longitude,5);
    }
  } else if (debugStream!=0 && gll.status == 'V' && debugLevel >= DEBUG_LEVEL_WARN) { debugStream->println(F("WARN : GLL is Void"));
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("ERROR: Failed to parse GLL")); }
}

void NMEA0183Gateway::HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {

  double TrueHeading;
  if (NMEA0183ParseHDT(NMEA0183Msg,TrueHeading)) {
    tN2kMsg N2kMsg;
    double MHeading = toMagnetic(TrueHeading,Variation);
    SetN2kMagneticHeading(N2kMsg,1,MHeading,0,Variation);
    pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: True heading=")); debugStream->println(TrueHeading);
    }
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("Failed to parse HDT")); }
}

void NMEA0183Gateway::HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG, COG, SOG;
  
  if (NMEA0183ParseVTG(NMEA0183Msg,COG,MagneticCOG,SOG)) {
    Variation=COG-MagneticCOG; // Save variation for Magnetic heading
    tN2kMsg N2kMsg;
    SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,COG,SOG);
    pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: COG=")); debugStream->println(COG);
    }
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("Failed to parse VTG")); }
}

/**
 * Receive NMEA0183 BOD message (Bearing Origin to Destination) and store it for later use when the RMB message is received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void NMEA0183Gateway::HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183ParseBOD(NMEA0183Msg,bod)) {
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: BOD: True heading=")); debugStream->println(bod.trueBearing);
      debugStream->print(F("TRACE: BOD: Magnetic heading=")); debugStream->println(bod.magBearing);
      debugStream->print(F("TRACE: BOD: Origin ID=")); debugStream->println(bod.originID.c_str());
      debugStream->print(F("TRACE: BOD: Dest ID=")); debugStream->println(bod.destID.c_str());
    }
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("Failed to parse BOD")); }
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
       route.wpListInProgress = true;
       route.routeID = rte.routeID;
       route.wpList.clear();
    }

    //Combine the waypoints of correlated RTE messages in a central ist.
    //Will be processed when the last RTE message is recevied.
    for (char* currWp : rte.wp) {
      std::string wp = currWp;
      route.wpList.push_back(wp);
    }

    if (rte.currSentence == rte.nrOfsentences) {
      if (route.wpList.size() == 1) {
        handleGarminGPS120GOTO(route, Latitude,Longitude);
      } else if (rte.type == 'c') {
        //No need to remove waypoints when rte.type == 'w'
        removeWaypointsUpToOriginCurrentLeg(route, bod.originID);
      }
      removeUnusedWaypointsFromRoute(route);
      route.wpListInProgress = false;
      //Sending PGN129285 is handled from a timer.     
    }

    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: RTE: Time=")); debugStream->println(millis());
      debugStream->print(F("TRACE: RTE: Nr of sentences=")); debugStream->println(rte.nrOfsentences);
      debugStream->print(F("TRACE: RTE: Current sentence=")); debugStream->println(rte.currSentence);
      debugStream->print(F("TRACE: RTE: Type=")); debugStream->println(rte.type);
      debugStream->print(F("TRACE: RTE: Route ID=")); debugStream->println(rte.routeID);
    }
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("Failed to parse RTE")); }
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
    route.wpMap[wpl.name] = wpl;
    if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE) {
      debugStream->print(F("TRACE: WPL: Time=")); debugStream->println(millis());
      debugStream->print(F("TRACE: WPL: Name=")); debugStream->println(route.wpMap[wpl.name].name.c_str());
      debugStream->print(F("TRACE: WPL: Latitude=")); debugStream->println(route.wpMap[wpl.name].latitude);
      debugStream->print(F("TRACE: WPL: Longitude=")); debugStream->println(route.wpMap[wpl.name].longitude);
    }
  } else if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR) { debugStream->println(F("Failed to parse WPL")); }
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

  if (debugStream!=0 && debugLevel >= DEBUG_LEVEL_DEBUG) {
      debugStream->print(F("DEBUG : Memory min: ")); debugStream->print(memoryMin);
      debugStream->print(F(" Handling NMEA0183 message ")); debugStream->println(NMEA0183Msg.MessageCode());
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
