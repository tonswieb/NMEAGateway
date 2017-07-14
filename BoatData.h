#ifndef _BoatData_H_
#define _BoatData_H_

#include <list>
#include <map>
#include <string>

typedef struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  
  double TrueHeading,SOG,COG,Variation,
         GPSTime,// Secs since midnight,
         Latitude, Longitude, Altitude, HDOP, GeoidalSeparation, DGPSAge;
  int GPSQualityIndicator, SatelliteCount, DGPSReferenceStationID;
  bool MOBActivated;

public:
  tBoatData() {
    TrueHeading=0;
    SOG=0;
    COG=0; 
    Variation=7.0;
    GPSTime=0;
    Altitude=0;
    HDOP=100000;
    DGPSAge=100000;
    DaysSince1970=0; 
    MOBActivated=false; 
    SatelliteCount=0; 
    DGPSReferenceStationID=0;
  };
};

typedef struct tNavData {
  //NMEA0183-RMB. Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
  tRMB rmb;
  //NMEA0183-BOD (Bearing Origin to Destination). Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
  tBOD bod;
  //List of RTE waypoints. Can have multiple RTE messages which are linked. So keep a external list which is cleaned upon the last RTE message.
  std::list<std::string> wp;
  //Map of waypoints. Updated with each NMEA0183-WPL message and cleared with the last NMEA0183-RTE message.
  std::map<std::string,tWPL> wpMap;
};

#endif // _BoatData_H_

