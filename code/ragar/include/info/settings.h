#ifndef settings_H
#define settings_H

#include <string>
#include "botSetup/vex.h"

extern bool motorErrorAutoCorrection;

extern bool useCurve;

extern std::string defaultDriveType;

extern std::string driveType;

extern vex::brakeType driveTrainBraking;

extern float axis1Sense;

extern float axis4Sense;

extern float velocityCap;

extern bool useCataSmart;

extern bool diagnostic;

extern bool trollAkshay;

extern bool useJarDrive;

extern bool hasRunSetup;

extern bool botLabMode;

extern int globalRefresh;

extern bool logEnabled;

extern bool runningProgram;

extern std::string PSOTy;

extern std::string teamColor;

extern bool colorSort;

extern bool isTurning;

#endif