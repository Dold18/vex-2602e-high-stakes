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
#include "Sensing/slip.h"
 
#include <DriveFuncs/PID.h>
#include <Sensing/colorSort.h>
#include <autos/callbacks.h>
#include <string>
#include <random>
#include <array>

 

 
void turnTest(){
  setup();

  float startingH = 0;


  chassis.set_heading(0);

  chassis.set_turn_constants(12, .63, .015, 6.06, 35);
  //chassis.set_turn_constants(12, .625, .0135, 6.05, 35);

  //chassis.set_turn_constants(12, .625, .014, 6.29, 37);
  //chassis.set_turn_constants(12, .665, .021, 6.24, 30);
  //chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_turn_exit_conditions(1, 300, 3000);


  float startTime = Brain.timer(msec);
  startingH = turnToHeadingC(45);
  std::string btime = floatToString(Brain.timer(msec)-startTime);
  std::string bheading = floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);

  wait(1.5,seconds);

  startTime = Brain.timer(msec);
  startingH = turnToHeadingC(135);
  btime = btime + " " + floatToString(Brain.timer(msec)-startTime);
  bheading = bheading + " " + floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);

  
  wait(1.5,seconds);
  startTime = Brain.timer(msec);
  startingH = turnToHeadingC(315);
  btime = btime + " " + floatToString(Brain.timer(msec)-startTime);
  bheading = bheading + " " + floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);

}

void turnTestold(){
  setup();


  chassis.set_heading(0);

  chassis.set_turn_constants(12, .63, .015, 6.06, 35);
  //chassis.set_turn_constants(12, .625, .0135, 6.05, 35);

  //chassis.set_turn_constants(12, .625, .014, 6.29, 37);
  //chassis.set_turn_constants(12, .665, .021, 6.24, 30);
  //chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_turn_exit_conditions(1, 300, 3000);


  float startTime = Brain.timer(msec);
  chassis.turn_to_angle(45);
  std::string btime = floatToString(Brain.timer(msec)-startTime);
  std::string bheading = floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);

  wait(1.5,seconds);

  startTime = Brain.timer(msec);
  chassis.turn_to_angle(135);
  btime = btime + " " + floatToString(Brain.timer(msec)-startTime);
  bheading = bheading + " " + floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);

  
  wait(1.5,seconds);
  startTime = Brain.timer(msec);
  chassis.turn_to_angle(315);
  btime = btime + " " + floatToString(Brain.timer(msec)-startTime);
  bheading = bheading + " " + floatToString(Inertial.heading());
  displayInfo(Controller1, btime, bheading);
}




void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0,50, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0,110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0,130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}



void pureTest(){
  logD("RUN TEST PURE", "Running pure test");

  setup();

  //odom_constants();

  float startTime = Brain.timer(seconds);

  chassis.set_coordinates(-24.107, -48.005, 00);

  peerPursuit(15000, 0, -1, 4);
  setMotorVelocities(0);
  displayInfo(Controller1, "C4", "", floatToString(Brain.timer(seconds)-startTime));
}


void lineTrackDriveTest(){
  logD("RUN LineTrack", "Running lineDrvie");

  setup();

  odom_constants();

  wait(15,msec);

  driveToLine(true, 2000);

  setMotorVelocities(0);
}




void testSensors(){
  logD("RUN testSensors", "Running testSensors");

  setup();
  while(1){
    Brain.Screen.clearScreen();
    //Brain.Screen.printAt(0,50, "D1 Left: %f", leftSonar.distance(inches));
    //Brain.Screen.printAt(0,70, "D2 Right: %f",  rightSonar.distance(inches));
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());

    Brain.Screen.printAt(0,110, "DistAngle: %f", distanceGetAngle());
    Brain.Screen.printAt(0,130, "DistYPos: %f", distanceGetY(distanceGetAngle()));
    task::sleep(20);
  }

}



void turnTestPoint(){

  logD("RUN TTP", "Running Turn Test using points");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 0);

  int startTime = Brain.timer(msec);

  chassis.set_drive_exit_conditions(1, 300, 3000);
  chassis.drive_to_point(0, 20);
  turnToHeadingC(90, "DOG");

  chassis.set_drive_exit_conditions(1, 300, 3000);
  chassis.drive_to_point(20, 20);
  turnToHeadingC(180, "DOG");

  chassis.set_drive_exit_conditions(1, 300, 3000);
  chassis.drive_to_point(20, 0);
  turnToHeadingC(270, "DOG");

  chassis.set_drive_exit_conditions(1, 300, 3000);
  chassis.drive_to_point(0, 0);

  logD("END TTP", "TIME:" + FTS(Brain.timer(msec)-startTime));
}

void turnTestReg(){

  logD("RUN TTR", "Running Turn Test regular funcs");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 0);

  int startTime = Brain.timer(msec);

  driveDistance(20);

  turnToHeadingC(90, "HIGH");

  driveDistance(20);

  turnToHeadingC(180, "HIGH");

  driveDistance(20);

  turnToHeadingC(270, "HIGH");

  driveDistance(20);

  logD("END TTR", "TIME:" + FTS(Brain.timer(msec)-startTime));

}




void accelTest(){
  logD("RUN ATT", "Running accel test");

  setup();

  testSlip();

  
}


void distanceTest(){
  logD("RUN testSensors", "Running testSensors");

  setup();
  while(1){
    Brain.Screen.clearScreen();
    //Brain.Screen.printAt(0,50, "D1 Left: %f", leftSonar.distance(inches));
    //Brain.Screen.printAt(0,70, "D2 Right: %f",  rightSonar.distance(inches));
    Brain.Screen.printAt(0,90, "Heading: %f", chassis.get_absolute_heading());
    if(Controller1.ButtonA.pressing()){
      getPosDist(true, chassis.get_absolute_heading(),50, 10);
    }
    Brain.Screen.printAt(0,110, "X Pos: %f", chassis.get_X_position());
    Brain.Screen.printAt(0,130, "Y Pos: %f", chassis.get_Y_position());
    task::sleep(20);
  }

}




void testColorSort(){

  logD("RUN pointRIGHTRingsSafe", "pointRIGHTRingsSafe");

  //setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(12, -60, 270);
  //chassis.desired_heading = 90;

  getPosDist(true, chassis.get_absolute_heading(), 50, 15);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  //wait(5,sec);
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;
  teamColor = "RED";


  needReset = true;
  tPosLB = 1;

  setTargetIntake(100);

  task dist = task(distTaskMainLowACRight);

  chassis.drive_max_voltage = 0;
  chassis.drive_timeout = 150000;
  donaldDuck(150);
  
}