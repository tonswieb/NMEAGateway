/* 
Log.h

Author: Ton Swieb

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#ifndef _Log_H_
#define _Log_H_

#include <Arduino.h>

#define DEBUG_LEVEL_TRACE 4
#define DEBUG_LEVEL_DEBUG 3
#define DEBUG_LEVEL_INFO 2
#define DEBUG_LEVEL_WARN 1
#define DEBUG_LEVEL_ERROR 0

//Some macros to make the logging systeem a bit less cumbersome
#define LOG_TRACE debugStream!=0 && debugLevel >= DEBUG_LEVEL_TRACE
#define LOG_DEBUG debugStream!=0 && debugLevel >= DEBUG_LEVEL_DEBUG
#define LOG_INFO debugStream!=0 && debugLevel >= DEBUG_LEVEL_INFO
#define LOG_WARN debugStream!=0 && debugLevel >= DEBUG_LEVEL_WARN
#define LOG_ERROR debugStream!=0 && debugLevel >= DEBUG_LEVEL_ERROR
#define log_P(...) debugStream->print(F(__VA_ARGS__))
#define logln_P(...) debugStream->println(F(__VA_ARGS__))
#define log(...) debugStream->print(__VA_ARGS__)
#define logln(...) debugStream->println(__VA_ARGS__)

#endif

