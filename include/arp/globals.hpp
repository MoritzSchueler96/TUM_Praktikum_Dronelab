/*
 *  globals.hpp
 *  
 *  Created on: 18 Dec 2021
 *      Author: MoritzSchueler96
 *  
 *  Header for global variables needed across all files.
 *  
 */

#ifndef GLOBALS_H
#define GLOBALS_H
///\brief This struct defines the usable global log levels
enum loglevel_e {logDEBUG, logERROR, logWARNING, logDEBUG1, logRELEASE, logDEBUG2, logINFO, logDEBUG3};
extern loglevel_e logLevel;

#endif /* ARDRONE_PRACTICALS_INCLUDE_GLOBALS_HPP_ */