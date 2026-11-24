#include "info/settings.h"
#include "botSetup/vex.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/controllerUI.h"
#include <string>
#include <iostream>
#include <sstream>

//overheat and disconnect error fixing
bool motorErrorAutoCorrection = false;

//curve for joysticks 
bool useCurve = false; 

//Resets to thing when not in drive mode
std::string defaultDriveType = "TANK"; 

//THREEAXIS, TANK, GTA
std::string driveType = defaultDriveType;

vex::brakeType driveTrainBraking = vex::coast;

//pct
float axis1Sense = 0.9;

float axis4Sense = 0.30;

//NEVER CHANGE
float velocityCap = 1;

//Wheither we use my smart controls or the manual controls that are 
bool useCataSmart = false;

//Shows temps and disconnects only, no other txt
bool diagnostic = false;

bool trollAkshay = false;

//Don't turn off ever
bool useJarDrive = true;

bool hasRunSetup = false;//run setupfunctions to prevent memory permissions error

//This is going to be a big thing for tuning PSO and stuff
bool botLabMode = false;

//global refresh speed independent from auton btw. In MS
//does not do ANYTHING
int globalRefresh = 10;

bool logEnabled = ( true )   && Brain.SDcard.isInserted();

//running a program
bool runningProgram = false;

std::string PSOTy = "DRIVE";

std::string teamColor = "GREEN";

bool colorSort = false;


//DOn't move ring if we turning
bool isTurning = false;