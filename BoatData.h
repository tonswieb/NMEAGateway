#ifndef _BoatData_H_
#define _BoatData_H_

#include <list>
#include <map>
#include <string>

typedef struct tBoatData {
  unsigned long DaysSince1970;   // Days since 1970-01-01
  double Variation, Latitude, Longitude;
};

typedef struct tNavData {
  //NMEA0183-BOD (Bearing Origin to Destination). Does not conatin enough information to send a NMEA200 message, but contains some elements required for a NMEA2000 message.
  tBOD bod;
  //A construction of a complete list of RTE waypoints in progress. RTE waypoints can be received in multiple messages depending on the number of waypoints.
  std::list<std::string> wpListInProgress;
  //A copy of the last complete list of RTE waypoints received.
  //We need this because PGN129285 messages need to be send at least every 10 seconds, but a complete RTE waypoint list can take much longer depending on the number of waypoints.
  std::list<std::string> wpListComplete;
  //Map of waypoints. Updated with each NMEA0183-WPL message and cleared with the last NMEA0183-RTE message.
  std::map<std::string,tWPL> wpMapInProgress;
  //A copy of the last complete map of waypoints (WPL messages received).
  std::map<std::string,tWPL> wpMapComplete;
  unsigned int routeID;
};

#endif // _BoatData_H_

