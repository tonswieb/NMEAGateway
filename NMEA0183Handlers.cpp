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
    Serial.print("freeMemory()=");
    Serial.println(freeMemory());
  }
}

double toMagnetic(double True, double Variation) {

  double magnetic=True-Variation;
  while (magnetic<0) magnetic+=PI_2;
  while (magnetic>=PI_2) magnetic-=PI_2;
  return magnetic;    
}

void sendPGN129285(const tRTE &rte) {
  
      tN2kMsg N2kMsg;
      bool nextMessage=true;
      int i=0;
      for (std::string currWp : pND->wp) {
        if (nextMessage) {
          N2kMsg.Clear();
          SetN2kPGN129285(N2kMsg,i, 1, rte.routeID, false, false, "Unknown");
          nextMessage=false;
        }
        //Continue adding waypoints as long as they fit within a single message
        tWPL wpl = pND->wpMap[currWp];
        if (NMEA0183HandlersDebugStream!=0) {
          NMEA0183HandlersDebugStream->print("129285:");
          NMEA0183HandlersDebugStream->print(wpl.name.c_str());NMEA0183HandlersDebugStream->print(",");
          NMEA0183HandlersDebugStream->print(wpl.latitude);NMEA0183HandlersDebugStream->print(",");
          NMEA0183HandlersDebugStream->print(wpl.longitude);NMEA0183HandlersDebugStream->print("\n");
        }
        
        if (!AppendN2kPGN129285(N2kMsg, i, currWp.c_str(), wpl.latitude, wpl.longitude)) {
          //TODO: Ensure that we do not drop this message when max. message size is received.
          pNMEA2000->SendMsg(N2kMsg); 
          nextMessage=true;
        }
        i++;
      }

      if(!nextMessage) {
        //Send the message that was still in progress.
        pNMEA2000->SendMsg(N2kMsg);       
      }
}

void sendSetN2kNavigationInfo() {
  
      tN2kMsg N2kMsg;

      int i=0;
      int originID=0;
      int destinationID=0;
      for (std::string currWp : pND->wp) {
        if (currWp == pND->originID) {
          originID = i;          
        }
        if (currWp == pND->destinationID) {
          destinationID = i;
        }
        i++;
      }
      
      //B&G Triton ignores etaTime and etaDays and calculates from the other info. So let's set them to 0 for now.
      double etaTime, etaDays = 0.0;
      double Mbtw = toMagnetic(pND->btw,pBD->Variation);
      //What is PerpendicularCrossed?
      SetN2kNavigationInfo(N2kMsg,1,pND->dtw,N2khr_magnetic,false,false,N2kdct_RhumbLine,etaTime,etaDays,
                          pND->magBearingOriginToDestination,
                          Mbtw,
                          originID,
                          destinationID,
                          pND->destLatitude,
                          pND->destLongitude,
                          pND->vmg);
      pNMEA2000->SendMsg(N2kMsg);
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("NAV: originID="); NMEA0183HandlersDebugStream->print(pND->originID.c_str());NMEA0183HandlersDebugStream->print(",");NMEA0183HandlersDebugStream->println(originID);
      NMEA0183HandlersDebugStream->print("RMC: destinationID="); NMEA0183HandlersDebugStream->print(pND->destinationID.c_str());NMEA0183HandlersDebugStream->print(",");NMEA0183HandlersDebugStream->println(destinationID);
      NMEA0183HandlersDebugStream->print("RMC: latitude="); NMEA0183HandlersDebugStream->println(pND->destLatitude,5);
      NMEA0183HandlersDebugStream->print("RMC: longitude="); NMEA0183HandlersDebugStream->println(pND->destLongitude,5);
      NMEA0183HandlersDebugStream->print("RMC: VMG="); NMEA0183HandlersDebugStream->println(pND->vmg);
      NMEA0183HandlersDebugStream->print("RMC: DTW="); NMEA0183HandlersDebugStream->println(pND->dtw);
      NMEA0183HandlersDebugStream->print("RMC: BTW (Current to Destination="); NMEA0183HandlersDebugStream->println(Mbtw);
      NMEA0183HandlersDebugStream->print("RMC: BTW (Orign to Desitination)="); NMEA0183HandlersDebugStream->println(pND->magBearingOriginToDestination);
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
    double xte;

  tRMB rmb;
  if (NMEA0183ParseRMB_nc(NMEA0183Msg, rmb)) {
    pND->originID = rmb.originID;
    pND->destinationID = rmb.destID;
    pND->destLatitude = rmb.latitude;
    pND->destLongitude = rmb.longitude;
    pND->dtw = rmb.dtw;
    pND->btw = rmb.btw;
    pND->vmg = rmb.vmg;
    
    if (pNMEA2000!=0) {
      tN2kMsg N2kMsg;
      SetN2kXTE(N2kMsg,1,N2kxtem_Autonomous, false, rmb.xte);
      pNMEA2000->SendMsg(N2kMsg);
    }
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RMB: XTE="); NMEA0183HandlersDebugStream->println(rmb.xte);
      NMEA0183HandlersDebugStream->print("RMB: DTW="); NMEA0183HandlersDebugStream->println(pND->dtw);
      NMEA0183HandlersDebugStream->print("RMB: BTW="); NMEA0183HandlersDebugStream->println(pND->btw);
      NMEA0183HandlersDebugStream->print("RMB: VMG="); NMEA0183HandlersDebugStream->println(pND->vmg);
      NMEA0183HandlersDebugStream->print("RMB: OriginID="); NMEA0183HandlersDebugStream->println(pND->originID.c_str());
      NMEA0183HandlersDebugStream->print("RMB: DestinationID="); NMEA0183HandlersDebugStream->println(pND->destinationID.c_str());
      NMEA0183HandlersDebugStream->print("RMB: Latittude="); NMEA0183HandlersDebugStream->println(pND->destLatitude,5);
      NMEA0183HandlersDebugStream->print("RMB: Longitude="); NMEA0183HandlersDebugStream->println(pND->destLongitude,5);
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

void HandleBOD(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;

  tBOD bod;  
  if (NMEA0183ParseBOD_nc(NMEA0183Msg,bod)) {
    //Cannot create NMEA2000 message from NMEA0183 BOD, need at 
    pND->magBearingOriginToDestination = bod.magBearing;
    pND->trueBearingOriginToDestination = bod.trueBearing;
    pND->originID = bod.originID;
    pND->destinationID = bod.destID;

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("BOD: True heading="); NMEA0183HandlersDebugStream->println(bod.trueBearing);
      NMEA0183HandlersDebugStream->print("BOD: Magnetic heading="); NMEA0183HandlersDebugStream->println(bod.magBearing);
      NMEA0183HandlersDebugStream->print("BOD: Origin ID="); NMEA0183HandlersDebugStream->println(bod.originID.c_str());
      NMEA0183HandlersDebugStream->print("BOD: Dest ID="); NMEA0183HandlersDebugStream->println(bod.destID.c_str());
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse BOD"); }
}

void HandleRTE(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;

  tRTE rte;
  if (NMEA0183ParseRTE_nc(NMEA0183Msg,rte)) {

    for (char* currWp : rte.wp) {
      std::string wp = currWp;
      pND->wp.push_back(wp);
    }

    if (rte.currSentence == rte.nrOfsentences) {
      //First send the route with all the waypoints, next the navigation information referring to the waypoints in the list.
      //We cannot send NavigationInfo standalone, because wp is cleared when RTE is received.
      sendPGN129285(rte);
      sendSetN2kNavigationInfo();
      //Cleanup dynamic memory and be ready for the next set of WPL messages
      pND->wpMap.clear();
      pND->wp.clear();
    }

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("RTE: Nr of sentences="); NMEA0183HandlersDebugStream->println(rte.nrOfsentences);
      NMEA0183HandlersDebugStream->print("RTE: Current sentence="); NMEA0183HandlersDebugStream->println(rte.currSentence);
      NMEA0183HandlersDebugStream->print("RTE: Type="); NMEA0183HandlersDebugStream->println(rte.type);
      NMEA0183HandlersDebugStream->print("RTE: Route ID="); NMEA0183HandlersDebugStream->println(rte.routeID);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RTE"); }
}

/**
 * Addd waypoints to the map and override old ones.
 */
void HandleWPL(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;

  tWPL wpl;
  if (NMEA0183ParseWPL_nc(NMEA0183Msg,wpl)) {
    pND->wpMap[wpl.name] = wpl;
    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("WPL: Name="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].name.c_str());
      NMEA0183HandlersDebugStream->print("WPL: Latitude="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].latitude);
      NMEA0183HandlersDebugStream->print("WPL: Longitude="); NMEA0183HandlersDebugStream->println(pND->wpMap[wpl.name].longitude);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse WPL"); }
}
