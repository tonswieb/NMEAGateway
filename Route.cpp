/* 
Route.cpp

Author: Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include "Route.h"


Route::Route(byte maxWpPerRoute, byte maxWpNameLength, Stream* debugStream, int debugLevel) {
  
  this->debugStream = debugStream;
  this->debugLevel = debugLevel;
  this->maxWpPerRoute = maxWpPerRoute;
  this->maxWpNameLength = maxWpNameLength;
  waypoints = new tRouteWaypoint[maxWpPerRoute];
  for(int i = 0; i < maxWpPerRoute; ++i) {
    waypoints[i].name = new char[maxWpNameLength + 1];
  }
}

void Route::initialize(const unsigned int ID) {
  
  routeID = ID;
  equalToPrevious = true;
  valid = false;
  size = 0;
  index = 0;
}

void Route::addWaypoint(const char* src) {
  
  if (index >= maxWpPerRoute) {
    if (LOG_WARN) {
      log_P("WARN : Maximum waypoints per route is reached. Ignoring waypoint: ");logln(src);
    }
  } else {
    /*
     * Assume the route received is equal to the previous route received until proven otherwise based on
     * comparing the waypoints names received and hence assume all coordinates are received as well 
     * in between the two received routes.
     */
    if (equalToPrevious == false || !equalName(src)) {
      addName(src);
      equalToPrevious = false;
    }
    index++;
  }    
}

void Route::finalize(const char type, const char* originID) {
  
  if (index != 1 && type == 'c') {
    setOriginCurrentLeg(originID);
  } else {
    //No need to find origin when rte.type == 'w', because the 1st wp is the origin
    indexOriginCurrentLeg = 0;
  }
  if (equalToPrevious) {
    //The RTE is marked as valid after receiving the same RTE sequence twice
    //and alle WPL messages in between with the coordinates.
    valid = true;
  }
  //Reset index for receiving WPL messages.
  size = index;
  index=0;
}

void Route::addCoordinates(const char* src, double latitude, double longitude) {
  
    if (index >= maxWpPerRoute) {
        if (LOG_WARN) {
          log_P("WARN : Maximum waypoints per route is reached. Ignoring waypoint: ");logln(src);
        }      
    } else if (equalName(src)) {
      waypoints[index].latitude = latitude;
      waypoints[index].longitude = longitude;
      if (LOG_TRACE) {
        log_P("TRACE: WPL: Time="); logln(millis());
        log_P("TRACE: WPL: Name="); logln(waypoints[index].name);
        log_P("TRACE: WPL: Latitude="); logln(waypoints[index].latitude);
        log_P("TRACE: WPL: Longitude="); logln(waypoints[index].longitude);
      }      
      index++;     
    } else {
      //Invalidate the route. Apparently the RTE messages are not in sync with the WPL messages.
      //Normally WPL messages are send in the same order as the order in the RTE messages.
      //Should be an exceptional case, for example when we cannot keep up. So let's wait for a new RTE message and try again.
      valid = false;
      if (LOG_WARN) {
        logln_P("WARN : The received WPL message is not in sync with the previously received RTE messages.");
      }
      if (LOG_DEBUG) {
        log_P("DEBUG: RTE: Name="); logln(waypoints[index].name);
        log_P("DEBUG: WPL: Name="); logln(src);
        log_P("DEBUG: RTL: Index="); logln(index);
      }      
    }
}

void Route::addName(const char* src) {

  strncpy(waypoints[index].name, src, maxWpNameLength);
  waypoints[index].name[maxWpNameLength]='\0';
}

boolean Route::equalName(const  char* src) {

  return strncmp(waypoints[index].name,src, maxWpNameLength) == 0;
}

boolean Route::equalName(const  char* src, const byte i) {

  return strncmp(waypoints[i].name,src, maxWpNameLength) == 0;
}

void Route::logWaypointNames() {
  
  log_P("TRACE: RTE equal to previous: ");logln(equalToPrevious);
  log_P("TRACE: Waypoints in Route list: ");
  for (byte i=0; i < index;i++) {
    log(waypoints[i].name);log_P(",");
  }
  log("\n");
}

void Route::setOriginCurrentLeg(const char* originID) {

  for (byte i=0; i < size; i++) {
    if (equalName(originID,i)) {
      return i;
    }
  }

  if (LOG_WARN) {
    logln_P("WARN : The origin of the leg not found in the waypoint list of the route.");    
  }
  return 0;
}
