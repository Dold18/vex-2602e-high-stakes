#ifndef log_H
#define log_H

#include <string>
#include "botSetup/vex.h"

extern std::string LOG;

extern std::string logName;

extern bool logPIDSteps;

extern bool PSOLogging;

extern int logCounter;

void logD(std::string header, std::string data="", bool save=false);



int startLog();

int logDTask();

#endif