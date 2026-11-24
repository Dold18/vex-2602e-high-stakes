#include "info/tuneAble.h"


float runCurve(float value){
  //See desmos for more info
  float preABS = ((value*value)/95)-1; 
  float negative = 1;
  if(preABS<0){
    preABS=0;
  }
  if(value<0){
    negative = -1;
  }
  
  return  preABS * negative;
}

float turnAccuracy = 1;
  //Turning required accuricy in degrees

float driveAccuracy = 1;
  //Drive accuricy in cm

float turnVelocityAuton = 50;
  //percent

float driveVelocityAuton = 65;
  //percent

float driveTurnVelocityAuton = 55;
  //pct

float zeroAccuracy = 2;
  //pct

float zeroAccuracyDrive = 15;
  //pct

double rotationToTravel = 23.1697;
  //multiply rotation by this value to get CM traveled
  //we went 47 inches at -2766 degrees
  //or 119.38cm
  

float timeoutTurn = 2;
  //seconds

float timeoutDrive = 4;
  //seconds

float smallDistValue = 25;
  //fake cm



float autonRefreshSpeed = 10;

//pct