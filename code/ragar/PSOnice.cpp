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

bool antiGen = false;


//Set By user
float targetIntakeVelocity = 0;

//Used by code
float intakeVelocity = 0;
void sample(){

//This line Detects an intake jam
if(fabs(Intake.velocity(rpm))<5)
{/* Code to be run on a jam */}

Intake.setVelocity(100, pct);

}

int intakeTask(){
  if(antiGen==true){
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