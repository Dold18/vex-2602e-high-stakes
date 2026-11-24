#include "autos/tests.h"
#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "DriveFuncs/custom.h" 
#include "info/settings.h"
#include "info/log.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "DriveFuncs/PurePursuit.h"
#include "Sensing/distance.h"
#include "nonDriveFuncs/LadyBrown.h"
#include "Sensing/colorSort.h"



int intakeForwards(){
  setTargetIntake(100);

  return 1;
}

int intakeBackwards(){
  setTargetIntake(100);

  return 1;
}


int setLB(){
  tPosLB = 1;
  return 1;
}


int blank(){
  logD("LOG", "BLANK CALL");
  return 1;
}

int clampTrue(){
  logD("LOG", "Set Clamp True");
  clampPistons.set(true);
  return 1;
}

int clampFalse(){
  logD("LOG", "Set Clamp False");
  clampPistons.set(false);
  return 1;
}

int distTask270(){
  checkForDistTask(270, false, 8000);
  return 0;
}

int distTaskMain(){
  distTask(false, 15000, 5);
  return 0;
}
int distTaskMainLowAC(){
  distTask(false, 60000, 15);
  return 0;
}
int distTaskMainLowACRight(){
  distTask(true, 60000, 15);
  return 0;
}

int lowerIntake(){
  dPistons.set(false);
  return 0;
}

int backwardsBar(){
  tPosLB=60;
  return 0;
}
int forwardsBar(){
  tPosLB=160;
  return 0;
}

int revLower(){
  reverseLower = true;
  return 0;
}

int loadLB(){
  tPosLB = 360-340;
  return 0;
}