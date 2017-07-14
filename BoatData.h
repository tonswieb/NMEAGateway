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

typedef struct tNavData{
  double magBearingOriginToDestination, trueBearingOriginToDestination, destLatitude, destLongitude, dtw, btw, vmg = 0;
  std::string originID;
  std::string destinationID;
  std::list<std::string> wp;
  std::map<std::string,tWPL> wpMap;
};

#endif // _BoatData_H_

