#ifndef _BoatData_H_
#define _BoatData_H_

#include <list>
#include <map>
#include <string>

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

typedef struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  double Variation, Latitude, Longitude;
  //NMEA0183-BOD (Bearing Origin to Destination). Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
  tBOD bod;
  tRoute routeComplete;
  tRoute routeInProgress;
};

#endif // _BoatData_H_

