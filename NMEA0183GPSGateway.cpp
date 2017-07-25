/* 
NMEA0183GPSGateway.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include "NMEA0183GPSGateway.h"

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

void removeWaypointsUpToOriginCurrentLeg(tRoute &route, const std::string originID) {

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
  }
}

NMEA0183GPSGateway::NMEA0183GPSGateway(tNMEA2000* pNMEA2000, Stream* gps, Stream* debugStream) {

  this->debugStream = debugStream;
  this->pNMEA2000 = pNMEA2000;
  NMEA0183_3.Begin(gps,3, 4800);
}

/**
 * Resend PGN129285 at least every 5 seconds.
 * B&G Triton requires a PGN129285 message around every 10 seconds otherwise display of the destinationID is cleared.
 * Depending on how fast an updated route is constructed we are sending the same or updated route.
 */
void NMEA0183GPSGateway::handleLoop() {

  pNMEA2000->ParseMessages();

  tNMEA0183Msg NMEA0183Msg;  
  while (NMEA0183_3.GetMessage(NMEA0183Msg)) {
    HandleNMEA0183Msg(NMEA0183Msg);
  }
  
  static unsigned long timeUpdated=millis();
  if (timeUpdated+5000 < millis()) {
    timeUpdated=millis();
    sendPGN129285(routeComplete);
  }
}

/**
 * 129283 - Cross Track Error
 * Category: Navigation
 * This PGN provides the magnitude of position error perpendicular to the desired course.
 */
void NMEA0183GPSGateway::sendPGN129283(const tRMB &rmb) {

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
void NMEA0183GPSGateway::sendPGN129284(const tRMB &rmb) {
  
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
    if (debugStream!=0) {
      debugStream->print("129284: originID="); debugStream->print(bod.originID.c_str());debugStream->print(",");debugStream->println(originID);
      debugStream->print("129284: destinationID="); debugStream->print(bod.destID.c_str());debugStream->print(",");debugStream->println(destinationID);
      debugStream->print("129284: latitude="); debugStream->println(rmb.latitude,5);
      debugStream->print("129284: longitude="); debugStream->println(rmb.longitude,5);
      debugStream->print("129284: ArrivalCircleEntered="); debugStream->println(ArrivalCircleEntered);
      debugStream->print("129284: VMG="); debugStream->println(rmb.vmg);
      debugStream->print("129284: DTW="); debugStream->println(rmb.dtw);
      debugStream->print("129284: BTW (Current to Destination="); debugStream->println(Mbtw);
      debugStream->print("129284: BTW (Orign to Desitination)="); debugStream->println(bod.magBearing);
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
void NMEA0183GPSGateway::sendPGN129285(tRoute &route) {
  
  tN2kMsg N2kMsg;
  int i=0;
  SetN2kPGN129285(N2kMsg,i, 1, route.routeID, false, false, "Unknown");

  for (std::string currWp : route.wpList) {
    //Continue adding waypoints as long as they fit within a single message
    tWPL wpl = route.wpMap[currWp];
    if (wpl.name == currWp) {
      if (debugStream!=0) {
        debugStream->print("129285:");
        debugStream->print(wpl.name.c_str());debugStream->print(",");
        debugStream->print(wpl.latitude);debugStream->print(",");
        debugStream->print(wpl.longitude);debugStream->print("\n");
      }
      if (!AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude)) {
        //Max. nr. of waypoints per message is reached.Send a message with all waypoints upto this one and start constructing a new message.
        pNMEA2000->SendMsg(N2kMsg); 
        N2kMsg.Clear();
        SetN2kPGN129285(N2kMsg,i, 1, route.routeID, false, false, "Unknown");
        //TODO: Check for the result of the Append, should not fail due to message size. Perhaps some other reason?
        AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude);
      }
    } else if (debugStream!=0)  {
        debugStream->print("129285: Skipping ");
        debugStream->print(currWp.c_str());debugStream->print("\n");
    }
    i++;
  }
  pNMEA2000->SendMsg(N2kMsg);       
}

// NMEA0183 message Handler functions
void NMEA0183GPSGateway::HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {

  tRMC rmc;
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,rmc) && rmc.status == 'A') {
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
    if (debugStream!=0) {
      debugStream->print("RMC: GPSTime="); debugStream->println(rmc.GPSTime);
      debugStream->print("RMC: Latitude="); debugStream->println(rmc.latitude,5);
      debugStream->print("RMC: Longitude="); debugStream->println(rmc.longitude,5);
      debugStream->print("RMC: COG="); debugStream->println(rmc.trueCOG);
      debugStream->print("RMC: SOG="); debugStream->println(rmc.SOG);
      debugStream->print("RMC: DaysSince1970="); debugStream->println(rmc.daysSince1970);
      debugStream->print("RMC: Variation="); debugStream->println(rmc.variation);
    }
  } else if (debugStream!=0 && rmc.status == 'V') { debugStream->println("RMC is Void");
  } else if (debugStream!=0 && rmc.status == 'V') { debugStream->println("Failed to parse RMC"); }
}

/**
 * Receive NMEA0183 RMB message (Recommended Navigation Data for GPS).
 * Contains enough information to send a NMEA2000 PGN129283 (XTE) message and NMEA2000 PGN129284 message.
 */
void NMEA0183GPSGateway::HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {

  tRMB rmb;
  if (NMEA0183ParseRMB_nc(NMEA0183Msg, rmb)  && rmb.status == 'A') {
    sendPGN129283(rmb);
    sendPGN129284(rmb);
    if (debugStream!=0) {
      debugStream->print("RMB: XTE="); debugStream->println(rmb.xte);
      debugStream->print("RMB: DTW="); debugStream->println(rmb.dtw);
      debugStream->print("RMB: BTW="); debugStream->println(rmb.btw);
      debugStream->print("RMB: VMG="); debugStream->println(rmb.vmg);
      debugStream->print("RMB: OriginID="); debugStream->println(rmb.originID.c_str());
      debugStream->print("RMB: DestinationID="); debugStream->println(rmb.destID.c_str());
      debugStream->print("RMB: Latittude="); debugStream->println(rmb.latitude,5);
      debugStream->print("RMB: Longitude="); debugStream->println(rmb.longitude,5);
    }
  } else if (debugStream!=0 && rmb.status == 'V') { debugStream->println("RMB is Void");
  } else if (debugStream!=0) { debugStream->println("Failed to parse RMB"); }
}

void NMEA0183GPSGateway::HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {

  tGGA gga;
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,gga) && gga.GPSQualityIndicator > 0) {
    tN2kMsg N2kMsg;
    //129029
    SetN2kGNSS(N2kMsg,1,DaysSince1970,gga.GPSTime,gga.latitude,gga.longitude,gga.altitude,
              N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(gga.GPSQualityIndicator),gga.satelliteCount,gga.HDOP,0,
              gga.geoidalSeparation,1,N2kGNSSt_GPS,gga.DGPSReferenceStationID,gga.DGPSAge
              );
    pNMEA2000->SendMsg(N2kMsg); 
    Latitude = gga.latitude;
    Longitude = gga.longitude;

    if (debugStream!=0) {
      debugStream->print("GGA: Time="); debugStream->println(gga.GPSTime);
      debugStream->print("GGA: Latitude="); debugStream->println(gga.latitude,5);
      debugStream->print("GGA: Longitude="); debugStream->println(gga.longitude,5);
      debugStream->print("GGA: Altitude="); debugStream->println(gga.altitude,1);
      debugStream->print("GGA: GPSQualityIndicator="); debugStream->println(gga.GPSQualityIndicator);
      debugStream->print("GGA: SatelliteCount="); debugStream->println(gga.satelliteCount);
      debugStream->print("GGA: HDOP="); debugStream->println(gga.HDOP);
      debugStream->print("GGA: GeoidalSeparation="); debugStream->println(gga.geoidalSeparation);
      debugStream->print("GGA: DGPSAge="); debugStream->println(gga.DGPSAge);
      debugStream->print("GGA: DGPSReferenceStationID="); debugStream->println(gga.DGPSReferenceStationID);
    }
  } else if (debugStream!=0 && gga.GPSQualityIndicator == 0) { debugStream->println("GGA invalid GPS fix.");
  } else if (debugStream!=0) { debugStream->println("Failed to parse GGA"); }
}

void NMEA0183GPSGateway::HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {

  tGLL gll;
  if (NMEA0183ParseGLL_nc(NMEA0183Msg,gll) && gll.status == 'A') {
    tN2kMsg N2kMsg;
    //PGN129025
    SetN2kLatLonRapid(N2kMsg, gll.latitude, gll.longitude);
    pNMEA2000->SendMsg(N2kMsg);
    Latitude = gll.latitude;
    Longitude = gll.longitude;

    if (debugStream!=0) {
      debugStream->print("GLL: Time="); debugStream->println(gll.GPSTime);
      debugStream->print("GLL: Latitude="); debugStream->println(gll.latitude,5);
      debugStream->print("GLL: Longitude="); debugStream->println(gll.longitude,5);
    }
  } else if (debugStream!=0 && gll.status == 'V') { debugStream->println("GLL is Void");
  } else if (debugStream!=0) { debugStream->println("Failed to parse GLL"); }
}

void NMEA0183GPSGateway::HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG, COG, SOG;
  
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,COG,MagneticCOG,SOG)) {
    Variation=COG-MagneticCOG; // Save variation for Magnetic heading
    tN2kMsg N2kMsg;
    SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,COG,SOG);
    pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0) {
      debugStream->print("COG="); debugStream->println(COG);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse VTG"); }
}

/**
 * Receive NMEA0183 BOD message (Bearing Origin to Destination) and store it for later use when the RMB message is received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void NMEA0183GPSGateway::HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183ParseBOD_nc(NMEA0183Msg,bod)) {
    if (debugStream!=0) {
      debugStream->print("BOD: True heading="); debugStream->println(bod.trueBearing);
      debugStream->print("BOD: Magnetic heading="); debugStream->println(bod.magBearing);
      debugStream->print("BOD: Origin ID="); debugStream->println(bod.originID.c_str());
      debugStream->print("BOD: Dest ID="); debugStream->println(bod.destID.c_str());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse BOD"); }
}

/**
 * Handle receiving NMEA0183 RTE message (route).
 * Based on the info which is processed after the last correlated RTE message is received a NMEA2000 messages is periodically sent using a timer.
 * A timer is used because (at least) B&G Triton requires a new PGN129285 around every 10 sec, but receiving RTE messages could take much longer 
 * on routes with more then 5 waypoints. So we need to sent the same PGN129285 multiple times in between.
 * For this we also need previously send WPL, messages. A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) followed 
 * by the RTE message(s) in the next cycle. So it can take a while for long routes before alle waypoints are received and finally the RTE messages.
 */
void NMEA0183GPSGateway::HandleRTE(const tNMEA0183Msg &NMEA0183Msg) {

  tRTE rte;
  if (NMEA0183ParseRTE_nc(NMEA0183Msg,rte)) {

    routeInProgress.routeID = rte.routeID;

    //Combine the waypoints of correlated RTE messages in a central ist.
    //Will be processed when the last RTE message is recevied.
    for (char* currWp : rte.wp) {
      std::string wp = currWp;
      routeInProgress.wpList.push_back(wp);
    }

    if (rte.currSentence == rte.nrOfsentences) {
      if (routeInProgress.wpList.size() == 1) {
        handleGarminGPS120GOTO(routeInProgress, Latitude,Longitude);
      } else if (rte.type == 'c') {
        //No need to remove waypoints when rte.type == 'w'
        removeWaypointsUpToOriginCurrentLeg(routeInProgress, bod.originID);
      }
      //Create a new complete list and map
      routeComplete.wpList.clear();
      routeComplete.wpList.splice(routeComplete.wpList.begin(),routeInProgress.wpList);
      routeComplete.wpMap.clear();
      routeComplete.wpMap.swap(routeInProgress.wpMap);
      //Sending PGN129285 is handled from a timer.     
    }

    if (debugStream!=0) {
      debugStream->print("RTE: Time="); debugStream->println(millis());
      debugStream->print("RTE: Nr of sentences="); debugStream->println(rte.nrOfsentences);
      debugStream->print("RTE: Current sentence="); debugStream->println(rte.currSentence);
      debugStream->print("RTE: Type="); debugStream->println(rte.type);
      debugStream->print("RTE: Route ID="); debugStream->println(rte.routeID);
      debugStream->print("freeMemory()="); debugStream->println(freeMemory());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse RTE"); }
}

/**
 * Receive NMEA0183 WPL message (Waypoint List) and store it for later use after all RTE messages are received.
 * A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) so it can take a while for long routes 
 * before alle waypoints are received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void NMEA0183GPSGateway::HandleWPL(const tNMEA0183Msg &NMEA0183Msg) {
  
  tWPL wpl;
  if (NMEA0183ParseWPL_nc(NMEA0183Msg,wpl)) {
    routeInProgress.wpMap[wpl.name] = wpl;
    if (debugStream!=0) {
      debugStream->print("WPL: Time="); debugStream->println(millis());
      debugStream->print("WPL: Name="); debugStream->println(routeInProgress.wpMap[wpl.name].name.c_str());
      debugStream->print("WPL: Latitude="); debugStream->println(routeInProgress.wpMap[wpl.name].latitude);
      debugStream->print("WPL: Longitude="); debugStream->println(routeInProgress.wpMap[wpl.name].longitude);
      debugStream->print("freeMemory()="); debugStream->println(freeMemory());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse WPL"); }
}

void NMEA0183GPSGateway::HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183Msg.IsMessageCode("GGA")) {
    HandleGGA(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("GGL")) {
    HandleGLL(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("RMB")) {
    HandleRMB(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("RMC")) {
    HandleRMC(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("WPL")) {
    HandleWPL(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("BOD")) {
    HandleBOD(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("VTG")) {
    HandleVTG(NMEA0183Msg);
  } else if (NMEA0183Msg.IsMessageCode("RTE")) {
    HandleRTE(NMEA0183Msg);
  }
}
