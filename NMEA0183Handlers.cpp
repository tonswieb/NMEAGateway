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
Stream* NMEA0183HandlersDebugStream=0;
//true = NMEA0183 BOD.type = 'w', false = NMEA0183 BOD.type = 'c'
bool sendWPlistFromOriginCurrentLeg=true;

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
  NMEA0183HandlersDebugStream=_stream;
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
 * Send the active route with all waypoints from the origin of the current leg and onwards.
 * So the waypoint that corresponds with the originID from the BOD message should be the 1st. The destinationID from the BOD message should be the 2nd. Etc.
 */
void sendPGN129285(const tRTE &rte) {
  
      tN2kMsg N2kMsg;
      int i=0;
      SetN2kPGN129285(N2kMsg,i, 1, rte.routeID, false, false, "Unknown");

      for (std::string currWp : pND->wp) {
        //Continue adding waypoints as long as they fit within a single message
        tWPL wpl = pND->wpMap[currWp];
        if (wpl.name == currWp) {
          if (NMEA0183HandlersDebugStream!=0) {
            NMEA0183HandlersDebugStream->print("129285:");
            NMEA0183HandlersDebugStream->print(wpl.name.c_str());NMEA0183HandlersDebugStream->print(",");
            NMEA0183HandlersDebugStream->print(wpl.latitude);NMEA0183HandlersDebugStream->print(",");
            NMEA0183HandlersDebugStream->print(wpl.longitude);NMEA0183HandlersDebugStream->print("\n");
          }
          if (!AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude)) {
            //Max. nr. of waypoints per message is reached.Send a message with all waypoints upto this one and start constructing a new message.
            pNMEA2000->SendMsg(N2kMsg); 
            N2kMsg.Clear();
            SetN2kPGN129285(N2kMsg,i, 1, rte.routeID, false, false, "Unknown");
            //TODO: Check for the result of the Append, should not fail due to message size. Perhaps some other reason?
            AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude);
          }
        } else if (NMEA0183HandlersDebugStream!=0)  {
            NMEA0183HandlersDebugStream->print("129285: Skipping ");
            NMEA0183HandlersDebugStream->print(currWp.c_str());NMEA0183HandlersDebugStream->print("\n");
        }
        i++;
      }
      pNMEA2000->SendMsg(N2kMsg);       
}

/*
 * Send the navigation information of the current leg of the route.
 * B&G Trition ignores OriginWaypointNumber and DestinationWaypointNumber values in this message.
 * It always takes the 2nd waypoint from PGN129285 as DestinationWaypoint.
 * Not sure if that is compliant with NMEA2000 or B&G Trition specific.
 */
void sendPGN129284() {
  
      tN2kMsg N2kMsg;

      int i=0;
      int originID=0;
      for (std::string currWp : pND->wp) {
        if (currWp == pND->bod.originID) {
          originID = i;
          break;
        }
        i++;
      }
      int destinationID=originID+1;
      
      //B&G Triton ignores etaTime and etaDays and calculates from the other info. So let's set them to 0 for now.
      double etaTime, etaDays = 0.0;
      double Mbtw = toMagnetic(pND->rmb.btw,pBD->Variation);
      //What is PerpendicularCrossed?
      SetN2kNavigationInfo(N2kMsg,1,
                          pND->rmb.dtw,
                          N2khr_magnetic,false,false,
                          N2kdct_RhumbLine,
                          etaTime,
                          etaDays,
                          pND->bod.magBearing,
                          Mbtw,
                          originID,
                          destinationID,
                          pND->rmb.latitude,
                          pND->rmb.longitude,
                          pND->rmb.vmg);
      pNMEA2000->SendMsg(N2kMsg);
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("129284: originID="); NMEA0183HandlersDebugStream->print(pND->bod.originID.c_str());NMEA0183HandlersDebugStream->print(",");NMEA0183HandlersDebugStream->println(originID);
      NMEA0183HandlersDebugStream->print("129284: destinationID="); NMEA0183HandlersDebugStream->print(pND->bod.destID.c_str());NMEA0183HandlersDebugStream->print(",");NMEA0183HandlersDebugStream->println(destinationID);
      NMEA0183HandlersDebugStream->print("129284: latitude="); NMEA0183HandlersDebugStream->println(pND->rmb.latitude,5);
      NMEA0183HandlersDebugStream->print("129284: longitude="); NMEA0183HandlersDebugStream->println(pND->rmb.longitude,5);
      NMEA0183HandlersDebugStream->print("129284: VMG="); NMEA0183HandlersDebugStream->println(pND->rmb.vmg);
      NMEA0183HandlersDebugStream->print("129284: DTW="); NMEA0183HandlersDebugStream->println(pND->rmb.dtw);
      NMEA0183HandlersDebugStream->print("129284: BTW (Current to Destination="); NMEA0183HandlersDebugStream->println(Mbtw);
      NMEA0183HandlersDebugStream->print("129284: BTW (Orign to Desitination)="); NMEA0183HandlersDebugStream->println(pND->bod.magBearing);
    }
}

// NMEA0183 message Handler functions

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,pBD->Variation)) {
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      double MCOG = toMagnetic(pBD->COG,pBD->Variation);
      SetN2kCOGSOGRapid(N2kMsg,1,N2khr_magnetic,MCOG,pBD->SOG); 
      pNMEA2000->SendMsg(N2kMsg);
      SetN2kLatLonRapid(N2kMsg,pBD->Latitude,pBD->Longitude);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RMC: GPSTime="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("RMC: Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("RMC: Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("RMC: COG="); NMEA0183HandlersDebugStream->println(pBD->COG);
      NMEA0183HandlersDebugStream->print("RMC: SOG="); NMEA0183HandlersDebugStream->println(pBD->SOG);
      NMEA0183HandlersDebugStream->print("RMC: DaysSince1970="); NMEA0183HandlersDebugStream->println(pBD->DaysSince1970);
      NMEA0183HandlersDebugStream->print("RMC: Variation="); NMEA0183HandlersDebugStream->println(pBD->Variation);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
}

void HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;

  if (NMEA0183ParseRMB_nc(NMEA0183Msg, pND->rmb)) {    
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      SetN2kXTE(N2kMsg,1,N2kxtem_Autonomous, false, pND->rmb.xte);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RMB: XTE="); NMEA0183HandlersDebugStream->println(pND->rmb.xte);
      NMEA0183HandlersDebugStream->print("RMB: DTW="); NMEA0183HandlersDebugStream->println(pND->rmb.dtw);
      NMEA0183HandlersDebugStream->print("RMB: BTW="); NMEA0183HandlersDebugStream->println(pND->rmb.btw);
      NMEA0183HandlersDebugStream->print("RMB: VMG="); NMEA0183HandlersDebugStream->println(pND->rmb.vmg);
      NMEA0183HandlersDebugStream->print("RMB: OriginID="); NMEA0183HandlersDebugStream->println(pND->rmb.originID.c_str());
      NMEA0183HandlersDebugStream->print("RMB: DestinationID="); NMEA0183HandlersDebugStream->println(pND->rmb.destID.c_str());
      NMEA0183HandlersDebugStream->print("RMB: Latittude="); NMEA0183HandlersDebugStream->println(pND->rmb.latitude,5);
      NMEA0183HandlersDebugStream->print("RMB: Longitude="); NMEA0183HandlersDebugStream->println(pND->rmb.longitude,5);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RMB"); }
}

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,pBD->SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      pNMEA2000->SendMsg(N2kMsg); 
    }

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("GGA: Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("GGA: Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("GGA: Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("GGA: Altitude="); NMEA0183HandlersDebugStream->println(pBD->Altitude,1);
      NMEA0183HandlersDebugStream->print("GGA: GPSQualityIndicator="); NMEA0183HandlersDebugStream->println(pBD->GPSQualityIndicator);
      NMEA0183HandlersDebugStream->print("GGA: SatelliteCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("GGA: HDOP="); NMEA0183HandlersDebugStream->println(pBD->HDOP);
      NMEA0183HandlersDebugStream->print("GGA: GeoidalSeparation="); NMEA0183HandlersDebugStream->println(pBD->GeoidalSeparation);
      NMEA0183HandlersDebugStream->print("GGA: DGPSAge="); NMEA0183HandlersDebugStream->println(pBD->DGPSAge);
      NMEA0183HandlersDebugStream->print("GGA: DGPSReferenceStationID="); NMEA0183HandlersDebugStream->println(pBD->DGPSReferenceStationID);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
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
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse HDT"); }
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
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
}

//Cannot create NMEA2000 message from NMEA0183 BOD, need at 
void HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {

  if (NMEA0183ParseBOD_nc(NMEA0183Msg,pND->bod)) {
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("BOD: True heading="); NMEA0183HandlersDebugStream->println(pND->bod.trueBearing);
      NMEA0183HandlersDebugStream->print("BOD: Magnetic heading="); NMEA0183HandlersDebugStream->println(pND->bod.magBearing);
      NMEA0183HandlersDebugStream->print("BOD: Origin ID="); NMEA0183HandlersDebugStream->println(pND->bod.originID.c_str());
      NMEA0183HandlersDebugStream->print("BOD: Dest ID="); NMEA0183HandlersDebugStream->println(pND->bod.destID.c_str());
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse BOD"); }
}

void HandleRTE(const tNMEA0183Msg &NMEA0183Msg) {

  tRTE rte;
  if (NMEA0183ParseRTE_nc(NMEA0183Msg,rte)) {

    for (char* currWp : rte.wp) {
      std::string wp = currWp;
      pND->wp.push_back(wp);
    }

    if (rte.currSentence == rte.nrOfsentences) {
      //First send the route with all the waypoints, next the navigation information referring to the current leg in the list.
      //We cannot send NavigationInfo standalone, because wp is cleared when RTE is received.

      //Handle GOTO by inserting the current position as first waypoint.
      if (pND->wp.size() == 1) {
        tWPL wpl;
        wpl.name = "Current";
        wpl.latitude = pBD->Latitude;
        wpl.longitude = pBD->Longitude;
        pND->wpMap[wpl.name] = wpl;
        pND->wp.insert(pND->wp.begin(),wpl.name);
      }

      if (rte.type == 'c' && sendWPlistFromOriginCurrentLeg) {
        //No need to remove waypoints when rte.type == 'w'
        removeWaypointsUpToOriginCurrentLeg();
      }
      sendPGN129285(rte);
      sendPGN129284();
      //Cleanup dynamic memory and be ready for the next set of WPL messages
      pND->wpMap.clear();
      pND->wp.clear();
    }

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RTE: Nr of sentences="); NMEA0183HandlersDebugStream->println(rte.nrOfsentences);
      NMEA0183HandlersDebugStream->print("RTE: Current sentence="); NMEA0183HandlersDebugStream->println(rte.currSentence);
      NMEA0183HandlersDebugStream->print("RTE: Type="); NMEA0183HandlersDebugStream->println(rte.type);
      NMEA0183HandlersDebugStream->print("RTE: Route ID="); NMEA0183HandlersDebugStream->println(rte.routeID);
      Serial.print("freeMemory()=");
      Serial.println(freeMemory());
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RTE"); }
}

/**
 * Addd waypoints to the map and override old ones.
 */
void HandleWPL(const tNMEA0183Msg &NMEA0183Msg) {
  
  tWPL wpl;
  if (NMEA0183ParseWPL_nc(NMEA0183Msg,wpl)) {
    pND->wpMap[wpl.name] = wpl;
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("WPL: Name="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].name.c_str());
      NMEA0183HandlersDebugStream->print("WPL: Latitude="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].latitude);
      NMEA0183HandlersDebugStream->print("WPL: Longitude="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].longitude);
      Serial.print("freeMemory()=");
      Serial.println(freeMemory());
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse WPL"); }
}
