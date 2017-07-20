/* 
NMEA0183Handlers.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include <MemoryFree.h>

#define PI_2 6.283185307179586476925286766559

typedef struct {
  char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg); 
} tNMEA0183Handler;

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg);
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleBOD(const tNMEA0183Msg &NMEA0183Msg);
void HandleRTE(const tNMEA0183Msg &NMEA0183Msg);
void HandleWPL(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
tNMEA2000 *pNMEA2000=0;
tBoatData *pBD=0;
tNavData *pND=0;
Stream* debugStream=0;

tNMEA0183Handler NMEA0183Handlers[]={
  {"GGA",&HandleGGA},
  {"HDT",&HandleHDT},
  {"VTG",&HandleVTG},
  {"RMC",&HandleRMC},
  {"RMB",&HandleRMB},
  {"BOD",&HandleBOD},
  {"RTE",&HandleRTE},
  {"WPL",&HandleWPL},
  {0,0}
};

void InitNMEA0183Handlers(tNMEA2000 *_NMEA2000, tBoatData *_BoatData, tNavData *_NavData) {
  pNMEA2000=_NMEA2000;
  pBD=_BoatData;
  pND=_NavData;
}

void DebugNMEA0183Handlers(Stream* _stream) {
  debugStream=_stream;
}

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  if (NMEA0183Handlers[iHandler].Code!=0) {
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
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
void handleGarminGPS120GOTO() {
    tWPL wpl;
    wpl.name = "CURRENT";
    wpl.latitude = pBD->Latitude;
    wpl.longitude = pBD->Longitude;
    pND->wpMap[wpl.name] = wpl;
    pND->wp.insert(pND->wp.begin(),wpl.name);

}

void removeWaypointsUpToOriginCurrentLeg() {

    bool originFound=false;
    std::list<std::string>::iterator it = pND->wp.begin();
    for (; it!=pND->wp.end(); ++it) {
      if (*it == pND->bod.originID) {
        originFound=true;
        break;
      }
    }
    if (originFound) {
      pND->wp.erase(pND->wp.begin(),it);
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
void sendPGN129285(const tRTE &rte) {

      if (pND->wp.size() == 1) {
        handleGarminGPS120GOTO();
      } else if (rte.type == 'c') {
        //No need to remove waypoints when rte.type == 'w'
        removeWaypointsUpToOriginCurrentLeg();
      }
  
      tN2kMsg N2kMsg;
      int i=0;
      SetN2kPGN129285(N2kMsg,i, 1, rte.routeID, false, false, "Unknown");

      for (std::string currWp : pND->wp) {
        //Continue adding waypoints as long as they fit within a single message
        tWPL wpl = pND->wpMap[currWp];
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
            SetN2kPGN129285(N2kMsg,i, 1, rte.routeID, false, false, "Unknown");
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
void sendPGN129284() {
  
      tN2kMsg N2kMsg;

      /*
       * PGN129285 only gives route/wp data ahead in the Active Route. So originID will always be 0 and destinationID will always be 1.
       * Unclear why these ID's need to be set in PGN129284. On B&G Triton displays other values are ignored anyway.
       */
      int originID=0;
      int destinationID=originID+1;
      
      //B&G Triton ignores the etaTime and etaDays values and does the calculation by itself. So let's leave them at 0 for now.
      double etaTime, etaDays = 0.0;
      double Mbtw = toMagnetic(pND->rmb.btw,pBD->Variation);
      bool ArrivalCircleEntered = pND->rmb.arrivalAlarm == 'A';
      //PerpendicularCrossed not calculated yet.
      //Need to calculate it based on current lat/long, pND->bod.magBearing and pND->rmb.lat/long
      bool PerpendicularCrossed = false;
      SetN2kNavigationInfo(N2kMsg,1,pND->rmb.dtw,N2khr_magnetic,PerpendicularCrossed,ArrivalCircleEntered,N2kdct_RhumbLine,etaTime,etaDays,
                          pND->bod.magBearing,Mbtw,originID,destinationID,pND->rmb.latitude,pND->rmb.longitude,pND->rmb.vmg);
      pNMEA2000->SendMsg(N2kMsg);
    if (debugStream!=0) {
      debugStream->print("129284: originID="); debugStream->print(pND->bod.originID.c_str());debugStream->print(",");debugStream->println(originID);
      debugStream->print("129284: destinationID="); debugStream->print(pND->bod.destID.c_str());debugStream->print(",");debugStream->println(destinationID);
      debugStream->print("129284: latitude="); debugStream->println(pND->rmb.latitude,5);
      debugStream->print("129284: longitude="); debugStream->println(pND->rmb.longitude,5);
      debugStream->print("129284: ArrivalCircleEntered="); debugStream->println(ArrivalCircleEntered);
      debugStream->print("129284: VMG="); debugStream->println(pND->rmb.vmg);
      debugStream->print("129284: DTW="); debugStream->println(pND->rmb.dtw);
      debugStream->print("129284: BTW (Current to Destination="); debugStream->println(Mbtw);
      debugStream->print("129284: BTW (Orign to Desitination)="); debugStream->println(pND->bod.magBearing);
    }
}

/**
 * 129283 - Cross Track Error
 * Category: Navigation
 * This PGN provides the magnitude of position error perpendicular to the desired course.
 */
void sendPGN129283() {

  tN2kMsg N2kMsg;
  SetN2kXTE(N2kMsg,1,N2kxtem_Autonomous, false, pND->rmb.xte);
  pNMEA2000->SendMsg(N2kMsg);
}

// NMEA0183 message Handler functions

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,pBD->Variation)) {
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      double MCOG = toMagnetic(pBD->COG,pBD->Variation);
      //PGN129026
      SetN2kCOGSOGRapid(N2kMsg,1,N2khr_magnetic,MCOG,pBD->SOG); 
      pNMEA2000->SendMsg(N2kMsg);
      //PGN129025
      SetN2kLatLonRapid(N2kMsg,pBD->Latitude,pBD->Longitude);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (debugStream!=0) {
      debugStream->print("RMC: GPSTime="); debugStream->println(pBD->GPSTime);
      debugStream->print("RMC: Latitude="); debugStream->println(pBD->Latitude,5);
      debugStream->print("RMC: Longitude="); debugStream->println(pBD->Longitude,5);
      debugStream->print("RMC: COG="); debugStream->println(pBD->COG);
      debugStream->print("RMC: SOG="); debugStream->println(pBD->SOG);
      debugStream->print("RMC: DaysSince1970="); debugStream->println(pBD->DaysSince1970);
      debugStream->print("RMC: Variation="); debugStream->println(pBD->Variation);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse RMC"); }
}

/**
 * Receive NMEA0183 RMB message (Recommended Navigation Data for GPS).
 * Contains enough information to send a NMEA2000 PGN129283 (XTE) message and NMEA2000 PGN129284 message.
 */
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;

  if (NMEA0183ParseRMB_nc(NMEA0183Msg, pND->rmb)) {    
    if (pNMEA2000!=0) {
      sendPGN129283();
      sendPGN129284();
    }
    if (debugStream!=0) {
      debugStream->print("RMB: XTE="); debugStream->println(pND->rmb.xte);
      debugStream->print("RMB: DTW="); debugStream->println(pND->rmb.dtw);
      debugStream->print("RMB: BTW="); debugStream->println(pND->rmb.btw);
      debugStream->print("RMB: VMG="); debugStream->println(pND->rmb.vmg);
      debugStream->print("RMB: OriginID="); debugStream->println(pND->rmb.originID.c_str());
      debugStream->print("RMB: DestinationID="); debugStream->println(pND->rmb.destID.c_str());
      debugStream->print("RMB: Latittude="); debugStream->println(pND->rmb.latitude,5);
      debugStream->print("RMB: Longitude="); debugStream->println(pND->rmb.longitude,5);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse RMB"); }
}

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,pBD->SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      //129029
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      pNMEA2000->SendMsg(N2kMsg); 
    }

    if (debugStream!=0) {
      debugStream->print("GGA: Time="); debugStream->println(pBD->GPSTime);
      debugStream->print("GGA: Latitude="); debugStream->println(pBD->Latitude,5);
      debugStream->print("GGA: Longitude="); debugStream->println(pBD->Longitude,5);
      debugStream->print("GGA: Altitude="); debugStream->println(pBD->Altitude,1);
      debugStream->print("GGA: GPSQualityIndicator="); debugStream->println(pBD->GPSQualityIndicator);
      debugStream->print("GGA: SatelliteCount="); debugStream->println(pBD->SatelliteCount);
      debugStream->print("GGA: HDOP="); debugStream->println(pBD->HDOP);
      debugStream->print("GGA: GeoidalSeparation="); debugStream->println(pBD->GeoidalSeparation);
      debugStream->print("GGA: DGPSAge="); debugStream->println(pBD->DGPSAge);
      debugStream->print("GGA: DGPSReferenceStationID="); debugStream->println(pBD->DGPSReferenceStationID);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse GGA"); }
}

void HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseHDT_nc(NMEA0183Msg,pBD->TrueHeading)) {
    if (pNMEA2000!=0) { 
      tN2kMsg N2kMsg;
      // Stupid Raymarine can not use true heading
      double MHeading = toMagnetic(pBD->TrueHeading,pBD->Variation);
      SetN2kMagneticHeading(N2kMsg,1,MHeading,0,pBD->Variation);
//      SetN2kTrueHeading(N2kMsg,1,pBD->TrueHeading);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (debugStream!=0) {
      debugStream->print("True heading="); debugStream->println(pBD->TrueHeading);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse HDT"); }
}

void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
 double MagneticCOG;
  if (pBD==0) return;
  
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
      pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading
    if (pNMEA2000!=0) { 
      tN2kMsg N2kMsg;
      SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
      pNMEA2000->SendMsg(N2kMsg);
      //SetN2kBoatSpeed(N2kMsg,1,SOG);
      //NMEA2000.SendMsg(N2kMsg);
    }
    if (debugStream!=0) {
      debugStream->print("True heading="); debugStream->println(pBD->TrueHeading);
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse VTG"); }
}

/**
 * Receive NMEA0183 BOD message (Bearing Origin to Destination) and store it for later use when the RMB message is received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183ParseBOD_nc(NMEA0183Msg,pND->bod)) {
    if (debugStream!=0) {
      debugStream->print("BOD: True heading="); debugStream->println(pND->bod.trueBearing);
      debugStream->print("BOD: Magnetic heading="); debugStream->println(pND->bod.magBearing);
      debugStream->print("BOD: Origin ID="); debugStream->println(pND->bod.originID.c_str());
      debugStream->print("BOD: Dest ID="); debugStream->println(pND->bod.destID.c_str());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse BOD"); }
}

/**
 * Handle receiving NMEA0183 RTE message (route).
 * NMEA2000 messages are sent when last correlated RTE message is received. We also need previously send WPL, messages.
 * A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) followed by the RTE message(s) in the next cycle.
 * So it can take a while for long routes before alle waypoints are received.
 */
void HandleRTE(const tNMEA0183Msg &NMEA0183Msg) {

  tRTE rte;
  if (NMEA0183ParseRTE_nc(NMEA0183Msg,rte)) {

    //Combine the waypoints of correlated RTE messages in a central ist.
    //Will be processed when the last RTE message is recevied.
    for (char* currWp : rte.wp) {
      std::string wp = currWp;
      pND->wp.push_back(wp);
    }

    if (rte.currSentence == rte.nrOfsentences) {
      sendPGN129285(rte);
      //Cleanup dynamic memory and be ready for the next set of WPL messages
      pND->wp.clear();
      pND->wpMap.clear();
    }

    if (debugStream!=0) {
      debugStream->print("RTE: Time="); debugStream->println(millis());
      debugStream->print("RTE: Nr of sentences="); debugStream->println(rte.nrOfsentences);
      debugStream->print("RTE: Current sentence="); debugStream->println(rte.currSentence);
      debugStream->print("RTE: Type="); debugStream->println(rte.type);
      debugStream->print("RTE: Route ID="); debugStream->println(rte.routeID);
      Serial.print("freeMemory()=");
      Serial.println(freeMemory());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse RTE"); }
}

/**
 * Receive NMEA0183 WPL message (Waypoint List) and store it for later use after all RTE messages are received.
 * A single WPL message is sent per NMEA0183 message cycle (RMC, RMB, GGA, WPL) so it can take a while for long routes 
 * before alle waypoints are received.
 * Does not contain enough information itself to send a single NMEA2000 message.
 */
void HandleWPL(const tNMEA0183Msg &NMEA0183Msg) {
  
  tWPL wpl;
  if (NMEA0183ParseWPL_nc(NMEA0183Msg,wpl)) {
    pND->wpMap[wpl.name] = wpl;
    if (debugStream!=0) {
      debugStream->print("WPL: Time="); debugStream->println(millis());
      debugStream->print("WPL: Name="); debugStream->println(pND->wpMap[wpl.name].name.c_str());
      debugStream->print("WPL: Latitude="); debugStream->println(pND->wpMap[wpl.name].latitude);
      debugStream->print("WPL: Longitude="); debugStream->println(pND->wpMap[wpl.name].longitude);
      Serial.print("freeMemory()=");
      Serial.println(freeMemory());
    }
  } else if (debugStream!=0) { debugStream->println("Failed to parse WPL"); }
}
