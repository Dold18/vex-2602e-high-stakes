#include "DriveFuncs/motorFuncs.h"
#include "botSetup/vex.h"
#include "info/settings.h"
#include "info/log.h"
#include "Libraries/ControllerUI.h"
#include "Libraries/stringFunctions.h"

void setMotorVelocities(float speed){
  LeftMotors.spin(vex::directionType::fwd);
  RightMotors.spin(vex::directionType::fwd); 

  LeftMotors.setVelocity(speed, pct);
  RightMotors.setVelocity(speed, pct);
}  

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  //chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_turn_constants(12, .439, .01436, 2.915, 14.55);
  //chassis.set_swing_constants(12, .3, .001, 2, 15); DEFAULKT
  chassis.set_swing_constants(12, .4695, 0.0292, 3.2157, 16);

  // Each exit condition set is in the form (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}


void odom_constants(){
  default_constants();
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
}

void setup() {
  if (hasRunSetup == false) {
    hasRunSetup = true;
    vexcodeInit();
    default_constants();
    
    ladyBrownTrack.setPosition(0, degrees);
    bMech.setPosition(0, degrees);
  }


  colorSensor.integrationTime(10);
  colorSensor.setLightPower(100, pct);

  setMotorVelocities(0);
  Intake.spin(vex::directionType::fwd);
  Intake.setVelocity(0, pct);
  

  bMech.setStopping(hold);
  bMech.spin(vex::directionType::fwd);
  bMech.setVelocity(0, pct);


  LeftMotors.setStopping(driveTrainBraking);
  RightMotors.setStopping(driveTrainBraking);
}






void driveToLine(bool forwards, int timeout){
  //elocity = fabs(velocity);
  logD("START DRIVETOLINE", "Starting drive to line forwards:" + BTS(forwards) + "timeout:" + FTS(timeout));
  bool crossedTheLine[2] = {false, false};
  bool blankSpaceTalyorSwift[2] = {false, false};
  bool lineDetected[2] ={false, false};
  int counter29 = 0;

  int reverse = forwards ? 1: -1;

  //extender name before
  vex::triport eyeCU =  Brain.ThreeWirePort;

  setMotorVelocities(8 * reverse);
  while(counter29<timeout){
    
    
    

    for(int i = 0; i<2; i++){
      //MAKE SURE TO UPDATE THESE
        lineDetected[i] = forwards ? (i==0 ? analog_in(eyeCU.H).value(percent)<10: analog_in(eyeCU.D).value(percent)<10)  :  (i==0 ? analog_in(eyeCU.E).value(percent)<10: analog_in(eyeCU.F).value(percent)<10);
      logD("STEP DRIVETOLINE", "Reverse: " + FTS(reverse) + ", LineDetected[0]:" + BTS(lineDetected[0]) + ", LineDetected[1]:" + BTS(lineDetected[1]) + ", blankSpaceTalyorSwift[0]:" + BTS(blankSpaceTalyorSwift[0]) + ", blankSpaceTalyorSwift[1]:" + BTS(blankSpaceTalyorSwift[1]) + ", crossedTheLine[0]:" + BTS(crossedTheLine[0]) + + ", crossedTheLine[1]:" + BTS(crossedTheLine[1]) + ", :" + FTS(analog_in(eyeCU.D).value(percent)) + ", :" + FTS(analog_in(eyeCU.H).value(percent)) );
      if(crossedTheLine[i] == false && lineDetected[i]){
        crossedTheLine[i] = true;
        logD("LOG DRIVETOLINE", "Motor Velocity set to 6");
        setMotorVelocities(6 * reverse);

        // logD("LOG DRIVETOLINE", "Motor Velocity set to 0");
        // setMotorVelocities(0);
        // logD("STOP DRIVETOLINE", "Stopping drive to line forwards:" + BTS(forwards) + "counter:" + FTS(counter29), true);
        // return;

      }
      else if(crossedTheLine[i] == true && blankSpaceTalyorSwift[i] == false && lineDetected[i] == false ){
        blankSpaceTalyorSwift[i] = true;
        //End in middle
        // logD("LOG DRIVETOLINE", "Motor Velocity set to 0");
        // setMotorVelocities(0);
        // return;
        logD("LOG DRIVETOLINE", "Motor Velocity set to idk nothing");
        //setMotorVelocities(8 * reverse);
      }
      else if(crossedTheLine[i] && blankSpaceTalyorSwift[i] && lineDetected[i]){
        logD("LOG DRIVETOLINE", "Motor Velocity set to 0");
        setMotorVelocities(0);
        return;
      }
    }

    counter29 += 10;
    
    wait(10,msec);
  }

  logD("STOP DRIVETOLINE", "Stopping drive to line forwards:" + BTS(forwards) + "counter:" + FTS(counter29), true);

}

bool cancelIntake = false;

int antiJamIntake(){
  int timeout = 15000;
  int startTime = Brain.timer(msec);

  bool jamReverse = false;
  int spinUp = 0;
  
  while((Brain.timer(msec) - startTime)<timeout){
    if(cancelIntake==true){
      return 0;
    }

    if(jamReverse == false){
      Intake.setVelocity(100, percent);
    }
    else{
      Intake.setVelocity(-100, percent);
    }

    if(spinUp>350){

      if(jamReverse==false && Intake.velocity(rpm) < 15){
        jamReverse=true;
        spinUp = 0;
      }
    }

    if(spinUp>150){

      if(jamReverse==true){
        jamReverse=false;
        spinUp=0;
      }
    }


    spinUp += 10;
    wait(10, msec);
    
  }
  logD("END", "Run", true);

  Intake.setVelocity(0, percent);

  return 0;
}



int intakeRing(){
  int timeout = 5000;
  int startTime = Brain.timer(msec);

  bool jamReverse = false;
  int spinUp = 0;
  
  while((Brain.timer(msec) - startTime)<timeout && ringDetector.objectDistance(mm)>50 ){
    if(cancelIntake==true){
      return 0;
    }
    if(jamReverse == false){
      Intake.setVelocity(100, percent);
    }
    else{
      Intake.setVelocity(-100, percent);
    }

    if(spinUp>500){

      if(jamReverse==false && Intake.velocity(rpm) < 15){
        jamReverse=true;
        spinUp = 0;
      }
    }

    if(spinUp>150){

      if(jamReverse==true){
        jamReverse=false;
        spinUp=0;
      }
    }


    spinUp += 10;
    wait(10, msec);
    
  }
  logD("END", "Run", true);

  Intake.setVelocity(0, percent);

  return 0;
}