#include "botSetup/vex.h"
#include "Libraries/botErrorProcesses.h"
#include "Libraries/stringFunctions.h"
#include "info/settings.h"
#include "info/log.h"
#include <string>

 
std::string getDisconnectErrors(){
  std::string errorLog = "Errs:";

  if(LadyBrown.installed()==false){ 
    errorLog = errorLog + " -LDY";
  } 
  if(Intake.installed()==false){
    errorLog = errorLog + " -Intk";
  }

  if(LeftBackMotor.installed()==false){
    errorLog = errorLog + " -LB";
  }
  if(LeftMiddleMotor.installed()==false){
    errorLog = errorLog + " -LM";
  }
  if(LeftFrontMotor.installed()==false){
    errorLog = errorLog + " -LFM";
  }

  if(RightBackMotor.installed()==false){
    errorLog = errorLog + " -RB";
  }
  if(RightMiddleMotor.installed()==false){
    errorLog = errorLog + " -RM";
  }
  if(RightFrontMotor.installed()==false){
    errorLog = errorLog + " -RF";
  }

  if(errorLog == std::string("Errs:")){
    errorLog = "";
    logD("LOG DISCONNECT", "NO DISCONNECTS");
  }
  else{
    logD("DISCONNECT", errorLog);
    errorLog = errorLog + " | ";
  }
  
  
  if(errorLog == std::string("")){
    errorLog = "";
  }

  
  return errorLog;
}

bool fixDriveMotors(float LeftMotors, float RightMotors){
  float LBMpct = 1;
  float LMMpct = 1;
  float LFMpct = 1;

  float RBMpct = 1;
  float RMMpct = 1;
  float RFMpct = 1;

  LeftFrontMotor.setStopping(driveTrainBraking);
  LeftMiddleMotor.setStopping(driveTrainBraking);
  LeftBackMotor.setStopping(driveTrainBraking);

  RightFrontMotor.setStopping(driveTrainBraking);
  RightMiddleMotor.setStopping(driveTrainBraking);
  RightBackMotor.setStopping(driveTrainBraking);

  bool badThings = false;
  //We are going to set the motor on the opposite side off if one motor is disconnected
  if(LeftBackMotor.installed()==false){
    badThings = true;
    RBMpct = 0;
    RightBackMotor.setStopping(coast);
  }
  if(LeftMiddleMotor.installed()==false){
    badThings = true;
    RMMpct = 0;
    RightMiddleMotor.setStopping(coast);
  }
  if(LeftFrontMotor.installed()==false){
    badThings = true;
    RFMpct = 0;
    RightFrontMotor.setStopping(coast);
  }

  if(RightBackMotor.installed()==false){
    badThings = true;
    LBMpct = 0;
    LeftBackMotor.setStopping(coast);
  }
  if(RightMiddleMotor.installed()==false){
    badThings = true;
    LMMpct = 0;
    LeftMiddleMotor.setStopping(coast);
  }
  if(RightFrontMotor.installed()==false){
    badThings = true;
    LFMpct = 0;
    LeftFrontMotor.setStopping(coast);
  }



//this is sorta trash for temp make too slow

  // if(LeftBackMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   RBMpct = 0.25;
  // }
  // else if(LeftBackMotor.temperature(celsius)==55){
  //   badThings = true;
  //   RBMpct = .5;
  // }

  // if(LeftMiddleMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   RMMpct = 0.25;
  // }
  // else if(LeftMiddleMotor.temperature(celsius)==55){
  //   badThings = true;
  //   RMMpct = .5;
  // }

  // if(LeftFrontMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   RFMpct = 0.25;
  // }
  // else if(LeftFrontMotor.temperature(celsius)==55){
  //   badThings = true;
  //   RFMpct = .5;
  // }



  // if(RightBackMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   LBMpct = 0.25;
  // }
  // else if(RightBackMotor.temperature(celsius)==55){
  //   badThings = true;
  //   LBMpct = .5;
  // }

  // if(RightMiddleMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   LMMpct = 0.25;
  // }
  // else if(RightMiddleMotor.temperature(celsius)==55){
  //   badThings = true;
  //   LMMpct = .5;
  // }

  // if(RightFrontMotor.temperature(celsius)>=60){
  //   badThings = true;
  //   LFMpct = 0.25;
  // }
  // else if(RightFrontMotor.temperature(celsius)==55){
  //   badThings = true;
  //   LFMpct = .5;
  // }

// LBMpct = 1;
// LMMpct = 1;
// LFMpct = 1;

// RBMpct = 1;
// RMMpct = 1;
// RFMpct = 1;

  LeftFrontMotor.setVelocity(LeftMotors * LFMpct, pct);
  LeftMiddleMotor.setVelocity(LeftMotors * LMMpct, pct);
  LeftBackMotor.setVelocity(LeftMotors * LBMpct, pct);

  RightFrontMotor.setVelocity(RightMotors * RFMpct, pct);
  RightMiddleMotor.setVelocity(RightMotors * RMMpct, pct);
  RightBackMotor.setVelocity(RightMotors * RBMpct, pct);
  

  return badThings;

}

std::string getTempMessage(){

  logD("LOG HEAT", "RF:" + FTS(RightFrontMotor.temperature(celsius)) + ", RM:" + FTS(RightMiddleMotor.temperature(celsius)) + ", RB:" + FTS(RightBackMotor.temperature(celsius)) + ", LF:" + FTS(LeftFrontMotor.temperature(celsius)) + ", LM:" + FTS(LeftMiddleMotor.temperature(celsius)) + ", LB:" + FTS(LeftBackMotor.temperature(celsius)) + ", Intake:" + FTS(Intake.temperature(celsius)));

  std::string errorLog = "";

  float maxTemp = 0;

  float RightMax = 0;
  if(RightFrontMotor.temperature(celsius)>RightMax){
    RightMax = RightFrontMotor.temperature(celsius);
  }
  if(RightMiddleMotor.temperature(celsius)>RightMax){
    RightMax = RightMiddleMotor.temperature(celsius);
  }
  if(RightBackMotor.temperature(celsius)>RightMax){
    RightMax = RightBackMotor.temperature(celsius);
  }
  


  float LeftMax = 0;
  if(LeftFrontMotor.temperature(celsius)>LeftMax){
    LeftMax = LeftFrontMotor.temperature(celsius);
  }
  if(LeftMiddleMotor.temperature(celsius)>LeftMax){
    LeftMax = LeftMiddleMotor.temperature(celsius);
  }
  if(LeftBackMotor.temperature(celsius)>LeftMax){
    LeftMax = LeftBackMotor.temperature(celsius);
  }



  if(RightMax >= 55 || LeftMax >= 55){
    if(RightMax>=LeftMax){
      errorLog = errorLog + "Crit Drive at " + floatToString(RightMax);
    }
    if(RightMax<LeftMax){
      errorLog = errorLog + "Crit Drive at " + floatToString(LeftMax);
    }
  }
  else if(RightMax >= 45 || LeftMax >= 45){
    if(RightMax>=LeftMax){
      errorLog = errorLog + "Warm Drive at " + floatToString(RightMax);
    }
    if(RightMax<LeftMax){
      errorLog = errorLog + "Warm Drive at " + floatToString(LeftMax);
    }
  }
  if(Intake.temperature(celsius)>=55){
    errorLog = errorLog + "Crit Intake at " + floatToString(Intake.temperature(celsius));
  }
  else if(Intake.temperature(celsius)==50){
    errorLog = errorLog + "Warm Intake at " + floatToString(Intake.temperature(celsius));
  }

  

  // if(Flywheel.temperature(celsius)>=55){
  //   errorLog = errorLog + "Crit Fly at " + floatToString(Flywheel.temperature(celsius));
  // }
  // else if(Flywheel.temperature(celsius)==50){
  //   errorLog = errorLog + "Warm Fly at " + floatToString(Flywheel.temperature(celsius));
  // }

  if(errorLog == std::string("")){  
    maxTemp = RightMax;
    errorLog = "RDrive";
    if(maxTemp < LeftMax){
      maxTemp = LeftMax;
      errorLog = "LDrive";
    }
    if(maxTemp < Intake.temperature(celsius)){
      maxTemp = Intake.temperature(celsius);
      errorLog = "Intake";
    }
    if(maxTemp < LadyBrown.temperature(celsius)){
      maxTemp = LadyBrown.temperature(celsius);
      errorLog = "LDY";
    }
    logD("LOG", "NO OVERHEATS");
    errorLog = "MxTmp: " + errorLog + " at "+ floatToString(maxTemp);
  }
  else{
    logD("OVERHEAT", errorLog);
  }

  
  return errorLog;
}

std::string getHeatMap(){
  logD("LOG HEAT", "RF:" + FTS(RightFrontMotor.temperature(celsius)) + ", RM:" + FTS(RightMiddleMotor.temperature(celsius)) + ", RB:" + FTS(RightBackMotor.temperature(celsius)) + ", LF:" + FTS(LeftFrontMotor.temperature(celsius)) + ", LM:" + FTS(LeftMiddleMotor.temperature(celsius)) + ", LB:" + FTS(LeftBackMotor.temperature(celsius)) + ", Intake:" + FTS(Intake.temperature(celsius)));

  std::string errorLog = "";

  float maxTemp = 0;

  if(RightFrontMotor.temperature(celsius)>=maxTemp){
    errorLog = "RF:" + floatToString(RightFrontMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = RightFrontMotor.temperature(celsius);
  }
  if(RightMiddleMotor.temperature(celsius)>=maxTemp){
    errorLog = "RM:" + floatToString(RightMiddleMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = RightMiddleMotor.temperature(celsius);
  }
  if(RightBackMotor.temperature(celsius)>=maxTemp){
    errorLog = "RB:" + floatToString(RightBackMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = RightBackMotor.temperature(celsius);
  }

  if(LeftFrontMotor.temperature(celsius)>=maxTemp){
    errorLog = "LF:" + floatToString(LeftFrontMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = LeftFrontMotor.temperature(celsius);
  }
  if(LeftMiddleMotor.temperature(celsius)>=maxTemp){
    errorLog = "LM:" + floatToString(LeftMiddleMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = LeftMiddleMotor.temperature(celsius);
  }
  if(LeftBackMotor.temperature(celsius)>=maxTemp){
    errorLog = "LB:" + floatToString(LeftBackMotor.temperature(celsius)) + " " + errorLog;
    maxTemp = LeftBackMotor.temperature(celsius);
  }

  if(Intake.temperature(celsius)>=maxTemp){
    errorLog = "Intk:" + floatToString(Intake.temperature(celsius)) + " " + errorLog;
    maxTemp = Intake.temperature(celsius);
  }
  if(LadyBrown.temperature(celsius)>=maxTemp){
    errorLog = "LDY:" + floatToString(LadyBrown.temperature(celsius)) + " " + errorLog;
    maxTemp = LadyBrown.temperature(celsius);
  }
  

  return errorLog;
}


bool intakeOverHeat(){
  logD("LOG HEAT", "RF:" + FTS(RightFrontMotor.temperature(celsius)) + ", RM:" + FTS(RightMiddleMotor.temperature(celsius)) + ", RB:" + FTS(RightBackMotor.temperature(celsius)) + ", LF:" + FTS(LeftFrontMotor.temperature(celsius)) + ", LM:" + FTS(LeftMiddleMotor.temperature(celsius)) + ", LB:" + FTS(LeftBackMotor.temperature(celsius)) + ", Intake:" + FTS(Intake.temperature(celsius)));

  float maxTemp = 0;

  if(RightFrontMotor.temperature(celsius)>=maxTemp){
    maxTemp = RightFrontMotor.temperature(celsius);
  }
  if(RightMiddleMotor.temperature(celsius)>=maxTemp){
    maxTemp = RightMiddleMotor.temperature(celsius);
  }
  if(RightBackMotor.temperature(celsius)>=maxTemp){
    maxTemp = RightBackMotor.temperature(celsius);
  }

  if(LeftFrontMotor.temperature(celsius)>=maxTemp){
    maxTemp = LeftFrontMotor.temperature(celsius);
  }
  if(LeftMiddleMotor.temperature(celsius)>=maxTemp){
    maxTemp = LeftMiddleMotor.temperature(celsius);
  }
  if(LeftBackMotor.temperature(celsius)>=maxTemp){
    maxTemp = LeftBackMotor.temperature(celsius);
  }

  if(maxTemp>=55){
    return true;
  }
  else{
    return false;
  }
  return false;
}

std::string getDiagnostic1(){
  logD("LOG HEAT", "RF:" + FTS(RightFrontMotor.temperature(celsius)) + ", RM:" + FTS(RightMiddleMotor.temperature(celsius)) + ", RB:" + FTS(RightBackMotor.temperature(celsius)) + ", LF:" + FTS(LeftFrontMotor.temperature(celsius)) + ", LM:" + FTS(LeftMiddleMotor.temperature(celsius)) + ", LB:" + FTS(LeftBackMotor.temperature(celsius)) + ", Intake:" + FTS(Intake.temperature(celsius)));

  std::string errorLog = "";
  if(LeftFrontMotor.installed()){
    errorLog = "LF:" + floatToString(LeftFrontMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LF: -1 " + errorLog;
  }
  if(LeftMiddleMotor.installed()){
    errorLog = "LM:" + floatToString(LeftMiddleMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LM: -1 " + errorLog;
  }
  if(LeftBackMotor.installed()){
    errorLog = "LB:" + floatToString(LeftBackMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LB: -1 " + errorLog;
  }

  return errorLog;
}

std::string getDiagnostic2(){
  std::string errorLog = "";
  if(RightFrontMotor.installed()){
    errorLog = "RF:" + floatToString(RightFrontMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RF: -1 " + errorLog;
  }
  if(RightMiddleMotor.installed()){
    errorLog = "RM:" + floatToString(RightMiddleMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RM: -1 " + errorLog;
  }
  if(RightBackMotor.installed()){
    errorLog = "RB:" + floatToString(RightBackMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RB: -1 " + errorLog;
  }
  return errorLog;
}

std::string getDiagnostic3(){
  std::string errorLog = "";
  errorLog = "Itk" + floatToString(Intake.temperature(celsius)) + " " + errorLog + " ";
  errorLog = "NODIS " + errorLog;
  errorLog = "LDY" + floatToString(LadyBrown.temperature(celsius)) + " " + errorLog;
  return errorLog;
}


//These are for the constant heat map that doesn't throw disconnect errors

std::string getHeat1(){
  std::string errorLog = "";
  if(LeftFrontMotor.installed()){
    errorLog = "LF:" + floatToString(LeftFrontMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LF: -1 " + errorLog;
  }
  if(LeftMiddleMotor.installed()){
    errorLog = "LM:" + floatToString(LeftMiddleMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LM: -1 " + errorLog;
  }
  if(LeftBackMotor.installed()){
    errorLog = "LB:" + floatToString(LeftBackMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LB: -1 " + errorLog;
  }

  return errorLog;
}

std::string getHeat2(){
  std::string errorLog = "";
  if(RightFrontMotor.installed()){
    errorLog = "RF:" + floatToString(RightFrontMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RF: -1 " + errorLog;
  }
  if(RightMiddleMotor.installed()){
    errorLog = "RM:" + floatToString(RightMiddleMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RM: -1 " + errorLog;
  }
  if(RightBackMotor.installed()){
    errorLog = "RB:" + floatToString(RightBackMotor.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "RB: -1 " + errorLog;
  }
  return errorLog;
}

std::string getHeat3(){
  std::string errorLog = "";
  if(Intake.installed()){
    errorLog = "Itk:" + floatToString(Intake.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "Itk: -1 " + errorLog;
  }

  errorLog = "    " + errorLog;

  if(LadyBrown.installed()){
    errorLog = "LDY:" + floatToString(LadyBrown.temperature(celsius)) + " " + errorLog;
  }
  else{
    errorLog = "LDY: -1 " + errorLog;
  }
  return errorLog;
}