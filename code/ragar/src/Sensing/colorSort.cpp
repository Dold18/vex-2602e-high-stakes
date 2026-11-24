#include "botSetup/vex.h"

#include <cmath>

#include <string>

#include "info/tuneAble.h"

#include "info/settings.h" 

#include "Libraries/controllerUI.h"

#include "Libraries/stringFunctions.h"

#include "botSetup/robot-config.h"

#include "info/log.h"

#include "JAR-Template/util.h"

#include "Sensing/colorSort.h"

#include "nonDriveFuncs/LadyBrown.h"

#include <info/colors.h>

float avgC = 0;
int avgCount = 0;

std::string deRing = "RED"; //"NONE"
std::string currentRing = deRing;


#include <iostream>
using namespace std;
 
// function to convert decimal to hexadecimal
string decToHexa(int n)
{
    // char array to store hexadecimal number
    char hexaDeciNum[2];
 
    // counter for hexadecimal number array
    int i = 0;
    while (n != 0) {
 
        // temporary variable to store remainder
        int temp = 0;
 
        // storing remainder in temp variable.
        temp = n % 16;
 
        // check if temp < 10
        if (temp < 10) {
            hexaDeciNum[i] = temp + 48;
            i++;
        }
        else {
            hexaDeciNum[i] = temp + 55;
            i++;
        }
 
        n = n / 16;
    }
 
    string hexCode = "";
    if (i == 2) {
        hexCode.push_back(hexaDeciNum[0]);
        hexCode.push_back(hexaDeciNum[1]);
    }
    else if (i == 1) {
        hexCode = "0";
        hexCode.push_back(hexaDeciNum[0]);
    }
    else if (i == 0)
        hexCode = "00";
 
    // Return the equivalent
    // hexadecimal color code
    return hexCode;
}
 
// Function to convert the
// RGB code to Hex color code
string convertRGBtoHex(int R, int G, int B)
{
    if ((R >= 0 && R <= 255)
        && (G >= 0 && G <= 255)
        && (B >= 0 && B <= 255)) {
 
        string hexCode = "#";
        hexCode += decToHexa(R);
        hexCode += decToHexa(G);
        hexCode += decToHexa(B);
 
        return hexCode;
    }
 
    // The hex color code doesn't exist
    else
        return "-1";
}

int redCount = 0;
int blueCount = 0;

int maxRed = 0;
int maxBlue = 0;
int maxClear = 0;

std::string getRingColor(){

  

  //logD("LOG RGB", "R:" + FTS(colorSensor.getRgb().red) + ", G:"+ FTS(colorSensor.getRgb().green)  + ", B:"+ FTS(colorSensor.getRgb().blue)  + ", C:"+ FTS(colorSensor.getRgb().clear) + ", avgC:"+ FTS(avgC) + ", index:"+ FTS(avgCount));
  if(true){//colorSensor.brightness()>0.04){// && (colorSensor.getRgb().red + colorSensor.getRgb().blue)>800 ){

    // if(colorSensor.getRgb().red>colorSensor.getRgb().blue){
    //   return "RED";
    // }
    // else{
    //   return "BLUE";
    // }
    // if(colorSensor.getRgb().red + 200 < colorSensor.getRgb().blue){
    //   return "BLUE";
    // }
    // if(colorSensor.getRgb().red<2){
    //   displayInfo(Controller1, "HELP");
    // }
    if(colorSensor.hue()>350 || colorSensor.hue()<60){
      redCount++;
      blueCount=0;
    }
    else if(colorSensor.getRgb().red + 200 < colorSensor.getRgb().blue || (colorSensor.hue()>120 && colorSensor.hue()<225)){
      blueCount++;
      redCount=0;
      return "BLUE";
    }
  }


  if(blueCount>1){
    return "BLUE";
  }
  if(redCount>1){
    return "RED";
  }
  

  return deRing;
}


float targetIntakeVelocity = 0;
float intakeVelocity = 0;
bool runColorSort = false;

bool EJECT = false;
int ejectCounter = 0;

// void eject(){
//   for(int i =0; i<20; i++){
//     if(getRingColor()!=teamColor && getRingColor()!="NONE"){
//       EJECT=true;
//       return;
//     }
//     wait(5,msec);
//   }
// }

// int colorSortTask(){

// }


bool intakeTask = false;

bool getNewColor = true;
bool switchPressed = false;

int counterSinceDetect = 0;



bool antiGen=true;
int timeStopped = 0;
bool runAnti = false;
int timeRunningAnti = 0;

//true for chain reverse, false for lb:
bool normalSwitcher = true;

//Just run lower section
bool runLower = false;
bool reverseLower = false;//Reverse lower
bool runTop = false;
bool stopAtSwitch = false;
bool realRunLower = false;

bool runLowerForward = false;




void setTargetIntake(float target){
  targetIntakeVelocity = target;
  intakeVelocity = target;
}

int runIntake(){
  colorSensor.integrationTime(5);
  intakeTask = true;
  wait(5,msec);

  while(intakeTask == true){
    intakeVelocity=targetIntakeVelocity;
    realRunLower = runLower;

    if(tPosLB<5){
      antiGen = true;
    }
    else{

      antiGen = false;
    }
    
    if(stopAtSwitch==true){
      if(ringDetector.objectDistance(mm)<100){
        realRunLower = true;
        setTargetIntake(0);
      }
    }



    if(colorSort && stopAtSwitch==false){
      if(colorSensor.getRgb().red > maxRed){
        maxRed = colorSensor.getRgb().red;
      }
      if(colorSensor.getRgb().blue > maxBlue){
        maxBlue = colorSensor.getRgb().blue;
      }

      std::string getRingCr = getRingColor();
      //logD("LOG COLORSORT", "Teamcolor:" + teamColor + ", currentRing:" + currentRing + ", getNewColor:" + BTS(getNewColor) + ", getRingColor:" + getRingCr  + ", SwitchPressed:" + BTS(switchPressed) + ", switchValue:" + FTS(ringSwitch.value()) + ", hue:" + FTS(colorSensor.hue()));
    //logD("LOG COLORSORT", FTS(colorSensor.getRgb().red) + ", " + FTS(colorSensor.getRgb().green) + ", " + FTS(colorSensor.getRgb().blue) + ", " + FTS(colorSensor.getRgb().clear));
      if(getNewColor == true && getRingCr != deRing){
        currentRing = getRingCr;
        
        getNewColor = false;
      }

      if(getNewColor == false){
        counterSinceDetect++;
      }
      else{
        counterSinceDetect = 0;
      }

      if(counterSinceDetect>80){
        getNewColor = true;
      }

      if(ringDetector.objectDistance(mm)<60){
        switchPressed=true;
      }

      if(normalSwitcher==true){

      if(ringDetector.objectDistance(mm)<35){//} && switchPressed==true){
        switchPressed = true;

        if(EJECT==false){
        EJECT = true;
            logD("LOG EJECT", "WE EJECTING, ring:" + currentRing);
        }
        // if(currentRing != "NONE" && currentRing != teamColor && teamColor!="GREEN" && teamColor!="NULL" && EJECT==false){ 
            
        // }
        // switchPressed = false;
        // getNewColor = true;
        // currentRing = deRing;
      }

      // if(switchPressed == true && ringSwitch.value()>0.5){
      //   if(currentRing != "NONE" && currentRing != teamColor && teamColor!="GREEN" && teamColor!="NULL" && EJECT==false){ 
      //       EJECT = true;
      //       logD("LOG EJECT", "WE EJECTING, ring:" + currentRing);
      //   }
      //   switchPressed = false;
      //   getNewColor = true;
      //   currentRing = "NONE";
      // }


      //Stop intake if turning
      // if(ringSwitch.value()<0.5 && isTurning==true && currentRing==teamColor){
      //   intakeVelocity=0;
      // }

      if(EJECT == true){
        int startEject = 0;
        // if(teamColor=="BLUE"){
        //   startEject=5;
        // }


//use 10 usuallly
        if(ejectCounter>= startEject && ejectCounter<= 15+startEject){
          if(maxRed>maxBlue+3){
            currentRing = "RED";
          }
          else if(maxBlue>maxRed+3){
            currentRing = "BLUE";
          }
          else{
            currentRing = "NONE";
          }

          // if(currentRing=="RED" && getNewColor==true){
          //   Controller1.rumble("-");
          // }
          // else if(currentRing=="RED"){
          //   Controller1.rumble(".");
          // }
          if(currentRing=="RED"){
            Controller1.rumble("-");
          }
          else if(currentRing=="BLUE"){
            Controller1.rumble(".");
          }

          //displayInfo(Controller1, FTS(colorSensor.getRgb().red) + ", " + FTS(colorSensor.getRgb().green), FTS(colorSensor.getRgb().blue) + ", " + FTS(colorSensor.getRgb().clear) );
          //wait(150, msec);



          if(currentRing != "NONE" && currentRing != teamColor && teamColor!="GREEN" && teamColor!="NULL"){
            intakeVelocity = -100;
            runLowerForward = true;
          }
          else {//if(ejectCounter> 2+startEject){
            ejectCounter=1000;
          }

        }
        else if(ejectCounter>startEject){
          
          runLowerForward = false;
          ejectCounter = -1;
          EJECT = false;
          intakeVelocity = targetIntakeVelocity;
          getNewColor=true;
          currentRing=deRing;
          blueCount=0;
          redCount=0;

          maxRed = 0;
          maxBlue = 0;
        }
        ejectCounter++;

      }
      }
      else{
        std::string getRingCr = getRingColor();
        if(currentRing != "NONE" && currentRing != teamColor && teamColor!="GREEN" && teamColor!="NULL"){ 
            tPosLB = 39;
        }
        else{
          tPosLB = 1;
        }
        if(getRingCr == teamColor){
          currentRing = teamColor;
        }
        else if(getRingCr!="NONE"){
          currentRing = getRingCr;
        }
      }
    }







    if(antiGen==true && realRunLower==false && isTurning==false){
          //displayInfo(Controller1, FTS(intakeVelocity) + FTS(Intake.velocity(rpm)));
      if(intakeVelocity>50 && fabs(Intake.velocity(rpm))<5){
        
        timeStopped++;
        if(timeStopped>13){
          runAnti = true;
          timeStopped=0;
          timeRunningAnti=0;
        }
      }
      else{
        timeStopped=0;
      }

      if(runAnti==true){
        timeRunningAnti++;

        intakeVelocity=-100;
        runLowerForward=true;

        if(timeRunningAnti>50){
          runAnti=false;
          timeStopped=0;
          timeRunningAnti=0;

          intakeVelocity=targetIntakeVelocity;
          runLowerForward=false;
        }
        
      }
      }
    
    if(realRunLower==true || isTurning==true){
      IntakeFull.stop();
      IntakeWatt5.spin(vex::forward);
      IntakeWatt5.setVelocity(intakeVelocity, pct);
    }
    else{
      if(runTop==true){
        Intake.spin(vex::forward);
        Intake.setVelocity(intakeVelocity, pct);
        IntakeWatt5.stop();
      }
      else{
        IntakeFull.spin(vex::forward);
        IntakeFull.setVelocity(intakeVelocity, pct);
      }
    }

    if(reverseLower==true){
      IntakeWatt5.spin(vex::forward);
      IntakeWatt5.setVelocity(-100, pct);
    }
    else if(runLowerForward){
      IntakeWatt5.spin(vex::forward);
      IntakeWatt5.setVelocity(100, pct);
    }

    wait(10,msec);
  }

  intakeTask = false;
  return 0;
}