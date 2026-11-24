#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "DriveFuncs/custom.h" 
#include "info/settings.h"
#include "info/log.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "DriveFuncs/PurePursuit.h"
#include "nonDriveFuncs/LadyBrown.h"
#include "DriveFuncs/PID.h"
#include "autos/callbacks.h"
#include "Sensing/colorSort.h"
#include "Sensing/distance.h"

#include <string>
#include <random>
#include <array>



//Blue, No alliance stake, 3 rings and then 1 center ring, scores on neutral stack with LB arm
void pointLeftRings(){

  logD("RUN pointRightRings", "Running Right Rings, points");

  setup();

  odom_constants();

  float xTrim = 0; 
  float yTrim = 0;

  chassis.set_coordinates(-37.75, -50, 209);
  chassis.desired_heading = 209;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;

  //wait(600,msec);
  //teamColor="RED";
  colorSort=true;
  
  
  tPosLB = 90;
  
  setDriveParams(16);
  ///chassis.drive_max_voltage = 6;
  donaldDuck(-19, 15);

  chassis.drive_max_voltage = 6;
  call4DD(-9, 5.5, clampTrue);

  setMotorVelocities(0);

  // chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  // chassis.set_heading_constants(6, .4, 0, 1, 0);

  clampPistons.set(true);

  setTargetIntake(100);

  wait(350,msec);

  turnToHeadingC(getAnglePoint(-36.075 + xTrim, -8 + yTrim), "MobileGoal0_LB90"); 
  //setDriveParams(getDistance(-36 + xTrim, -11 + yTrim));
  chassis.drive_to_point(-36.075 + xTrim, -8 + yTrim);

  setMotorVelocities(0);

  wait(200,msec);

  turnToHeadingC(getAnglePoint(-47 + xTrim, -19 + yTrim), "MobileGoal0_LB90"); 
  //setDriveParams(getDistance(-48 + xTrim, -24 + yTrim));
  chassis.drive_to_point(-47 + xTrim, -19 + yTrim);
  
  setMotorVelocities(0);
  wait(450,msec);

  task dist = task(distTaskMain);

  isTurning=true;
  turnToHeadingC(getAnglePoint(-51 + xTrim, -8 + yTrim), "MobileGoal2_LB90"); 
  isTurning=false;
  //setDriveParams(getDistance(-51 + xTrim, -10 + yTrim));
  chassis.drive_to_point(-51 + xTrim, -8 + yTrim);

  setMotorVelocities(0);
  wait(200,msec);

  driveDistance(-5);



  // chassis.drive_timeout = 1000;

  // turnToHeadingC(getAnglePoint(-66 + xTrim, -8 + yTrim), "MobileGoal2_LB90"); 
  // //setDriveParams(getDistance(-63 + xTrim, -7.5 + yTrim));
  // chassis.drive_to_point(-66 + xTrim, -8 + yTrim);

  // turnToHeadingC(getAnglePoint(-70 + xTrim, -1.2 + yTrim), "MobileGoal2_LB90"); 

  // //setMotorVelocities(12);
  // tPosLB = 180;
  // wait(860,msec);
  // // setMotorVelocities(20);
  // // wait(250,msec);
  // // setMotorVelocities(-10);
  // // wait(300,msec);
  // tPosLB = 1;


  // driveDistance(-16);


  //tPosLB=25;
  raiseIntake.set(true);

  turnToHeadingC(getAnglePoint(-8 + xTrim, -42 + yTrim), "MobileGoal3"); 
  driveDistance(getDistance(-8 + xTrim, -42 + yTrim)-4);
  driveDistance(8);

  raiseIntake.set(false);
  wait(650,msec);
  setTargetIntake(0);
  //turnToHeadingC(getAnglePoint(0 + xTrim, -68 + yTrim), "MobileGoal3"); 
  //driveDistance(18);

  raiseIntake.set(false);


  //chassis.drive_to_point(-62.5 + xTrim, -7.5 + yTrim);

  
}
//Direct mirror, lk shit
void pointGoalSideLeftMIRROR(){
  logD("RUN pointLeftGoal", "Running Left goal, points");

  setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(-24, -53, 180);
  chassis.desired_heading = 180;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  task dist = task(distTaskMainLowAC);




  driveDistance(-(getDistance(-24,-24)-9));

  call4DD(-8, 5, clampTrue);
  clampPistons.set(true);

  driveDistance(10);

  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  //wait(2,sec);

  turnToHeadingC(getAnglePoint(-11,4), "ULTRA");

  //chassis.drive_to_point(-4, 4);

  setTargetIntake(100);
  runTop = true;

  driveDistance( (getDistance(-11,4)-12.4) );

  //chassis.drive_to_point(14, -13);

  dPistons.set(true);

  wait(300,msec);

  reverseLower = true;

  driveDistance(-38);


  reverseLower = false;
  runTop = false;
  runLower=true;

  dPistons.set(false);

  wait(100,msec);

  turnToHeadingC(chassis.get_absolute_heading()+23);

  driveDistance(12);

  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  //wait(2,sec);

  turnToHeadingC(getAnglePoint(-48, -24));

  runLower=false;

  driveDistance(getDistance(-48, -24) - 4);

  turnToHeadingC(getAnglePoint(-48, -48));

  // call4DD(getDistance(-48, -48)+4, 5, revLower);

  // driveDistance(-5);

  call4DD(getDistance(-48, -48), 5, blank);


  turnToHeadingC(getAnglePoint(-68, -68));

  raiseIntake.set(true);

  reverseLower = false;

  setDriveParams(getDistance(-65, -65));

  chassis.drive_timeout=800;

  chassis.drive_distance(getDistance(-65, -65));

  raiseIntake.set(false);

  wait(250,msec);


  driveDistance(-8);

  // driveDistance(10);

  // driveDistance(-8);

  // driveDistance(10);

  // driveDistance(-8);

  // driveDistance(10);

  // driveDistance(-8);

  turnToHeadingC(-245, "DOG");
  clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);


  LeftMotors.setStopping(hold);
  RightMotors.setStopping(hold);
  driveDistance(-40);


}

//
// STUFF WE USE BELOW
//



//ELIMS 4 rings on 1 mogoal
void pointGoalSideRight(){
  logD("RUN pointRightRings", "Running Right Rings, points");

  setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(24, -53, 180);
  chassis.desired_heading = 180;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  task dist = task(distTaskMainLowACRight);




  driveDistance(-(getDistance(24,-24)-10));

  call4DD(-8, 5, clampTrue);
  clampPistons.set(true);

  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  //wait(2,sec);

  setTargetIntake(100);
  wait(30,msec);


  // turnToHeadingC(getAnglePoint(1.5,-4), "ULTRA");

  // //chassis.drive_to_point(-4, 4);

  // setTargetIntake(100);
  // runTop = true;

  // driveDistance( (getDistance(1.5,-4)-10) );

  // //chassis.drive_to_point(14, -13);

  // dPistons.set(true);

  // wait(300,msec);

  // reverseLower = true;

  // driveDistance(-23);


  // reverseLower = false;
  // runTop = false;
  // runLower=true;

  // dPistons.set(false);

  // wait(100,msec);

  // turnToHeadingC(chassis.get_absolute_heading()+23);

  // driveDistance(12);

  turnToHeadingC(getAnglePoint(48, -24));

  runLower=false;

  driveDistance(getDistance(48, -24) - 4);

  turnToHeadingC(getAnglePoint(48, -48));

  call4DD(getDistance(48, -48)+4, 5, revLower);

  driveDistance(-5);

  turnToHeadingC(getAnglePoint(68, -68));

  raiseIntake.set(true);

  reverseLower = false;

  setDriveParams(getDistance(65, -65));

  chassis.drive_timeout=1000;

  chassis.drive_distance(getDistance(65, -65));

  wait(100,msec);

  raiseIntake.set(false);

  

  driveDistance(-2);

  wait(1400,msec);


  driveDistance(-10);


  turnToHeadingC(245, "DOG");
  //clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);


  LeftMotors.setStopping(hold);
  RightMotors.setStopping(hold);
    //driveDistance(-30);


}
void pointGoalSideLeft(){
  logD("RUN pointLeftGoal", "Running Left goal, points");

  //setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(-24, -53, 180);
  chassis.desired_heading = 180;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  //vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  task dist = task(distTaskMainLowAC);

  // clampPistons.set(true);
  // setTargetIntake(100);

  // chassis.set_drive_constants(0.3, 1, 1, 1, 1);
  // chassis.drive_timeout=100000;
  // chassis.drive_distance(1000);

  
  // wait(50,sec);

  driveDistance(-(getDistance(-24,-24)-10));

  call4DD(-9, 6, clampTrue);
  clampPistons.set(true);

  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  //wait(2,sec);

  turnToHeadingC(getAnglePoint(-48, -24));

  runLower=false;
  setTargetIntake(100);

  driveDistance(getDistance(-48, -24) - 4);

  wait(800,msec);

  turnToHeadingC(getAnglePoint(-48, -48));

  // call4DD(getDistance(-48, -48)+4, 5, revLower);

  // driveDistance(-5);

  reverseLower = true;

  call4DD(getDistance(-48, -48), 5, blank);


  turnToHeadingC(getAnglePoint(-68, -68));

  raiseIntake.set(true);

  antiGen = false;

  reverseLower = false;

  setDriveParams(getDistance(-65, -65));

  chassis.drive_timeout=800;

  chassis.drive_distance(getDistance(-67, -67));

  raiseIntake.set(false);
  wait(100,msec);
  driveDistance(-2);

  wait(1500,msec);

  //chassis.drive_timeout=700;

  

  //wait(200,msec);

  driveDistance(-24);
  
  wait(400,msec);

  turnToHeadingC(-245, "DOG");
  //clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);


  LeftMotors.setStopping(hold);
  RightMotors.setStopping(hold);
  //driveDistance(-30);

  antiGen = true;


}
void pointGoalSideLeftOLD(){
  logD("RUN pointLeftGoal", "Running Left goal, points");

  //setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(-24, -53, 180);
  chassis.desired_heading = 180;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  //vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  task dist = task(distTaskMainLowAC);

  // clampPistons.set(true);
  // setTargetIntake(100);

  // chassis.set_drive_constants(0.3, 1, 1, 1, 1);
  // chassis.drive_timeout=100000;
  // chassis.drive_distance(1000);

  
  // wait(50,sec);

  driveDistance(-(getDistance(-24,-24)-10));

  call4DD(-9, 6, clampTrue);
  clampPistons.set(true);

  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  //wait(2,sec);

  turnToHeadingC(getAnglePoint(-48, -24));

  runLower=false;
  setTargetIntake(100);

  driveDistance(getDistance(-48, -24) - 4);

  turnToHeadingC(getAnglePoint(-48, -48));

  // call4DD(getDistance(-48, -48)+4, 5, revLower);

  // driveDistance(-5);

  call4DD(getDistance(-48, -48), 5, blank);


  turnToHeadingC(getAnglePoint(-68, -68));

  raiseIntake.set(true);

  antiGen = false;

  reverseLower = false;

  setDriveParams(getDistance(-65, -65));

  chassis.drive_timeout=800;

  chassis.drive_distance(getDistance(-67, -67));

  raiseIntake.set(false);

  wait(200,msec);

  chassis.drive_timeout=500;

  chassis.drive_distance(-8);

  chassis.drive_distance(11);

  chassis.drive_distance(-8);

  chassis.drive_distance(11);

  chassis.drive_distance(-8);

  chassis.drive_distance(11);

  wait(200,msec);

  chassis.drive_distance(-8);
  
  wait(400,msec);

  turnToHeadingC(-245, "DOG");
  clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);


  LeftMotors.setStopping(hold);
  RightMotors.setStopping(hold);
  driveDistance(-40);

  antiGen = true;


}

//ELIMS 1 alliance stake plus 3 rings on 1 mogoal
void pointLeftGoalElimWall(){
  logD("RUN pointRIGHTRingsSafe", "pointRIGHTRingsSafe");

  int startTime = Brain.Timer.time(sec);

  //setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(-10, -60, 90);
  //chassis.desired_heading = 90;

  //getPosDist(false, chassis.get_absolute_heading(), 50, 15);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  //wait(5,sec);
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  
  turnToHeadingC(getAnglePoint(0 + xTrim, -70.8 + yTrim), "PLAIN");

  tPosLB=180;

  wait(500,msec);

  driveDistance(-6);
  setMotorVelocities(0);
  tPosLB=1;

  wait(100,msec);

  turnToHeadingC(getAnglePoint(-0 + xTrim, -47 + yTrim), "PLAIN");

  raiseIntake.set(true);

  setTargetIntake(100);

  runLower=true;

  call4DD(5.5, 4, lowerIntake);
  setMotorVelocities(0);
  
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);

  raiseIntake.set(false);

  wait(100, msec);

  //driveDistance(-4);

  //wait(100,msec);

  task dist = task(distTaskMainLowAC);

  turnToHeadingC(180+getAnglePoint(-24 + xTrim, -24 + yTrim), "ULTRA");

  //wait(500,sec);

  //turnToHeadingC(180+getAnglePoint(26.5 + xTrim, -24 + yTrim), "ULTRA");
  // chassis.set_turn_constants(12, 0.4897, 0.0155, 3.10, 17.2966);
  // chassis.set_turn_exit_conditions(0.8, 500, 1000);

  // chassis.turn_to_angle(180+getAnglePoint(-24 + xTrim, -24 + yTrim));
  //wait(2, sec);

  //default_constants();
  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  
  driveDistance(-getDistance(-24, -24)+17.5);
  setMotorVelocities(0);
  
  

  //turnToHeadingC(180+getAnglePoint(-24 + xTrim, -24 + yTrim), "PLAIN");

  


  driveDistance(-8);

  call4DD(-8, 5, clampTrue);

  setTargetIntake(0);

  clampPistons.set(true);




  turnToHeadingC(getAnglePoint(-48, -24));

  setTargetIntake(100);
  runLower=false;

  driveDistance(getDistance(-48, -24) - 8);

  turnToHeadingC(-145);

  

  setDriveParams(80);

  chassis.drive_timeout=500;

  chassis.drive_distance(80);

  raiseIntake.set(true);

  chassis.drive_timeout=900;

  chassis.drive_distance(40);

  raiseIntake.set(false);

  donaldDuck(-2);

  wait(1800,msec);

  turnToHeadingC(-245, "DOG");
  clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);

  if((Brain.Timer.time(sec) - startTime) < 14.1){
    LeftMotors.setStopping(hold);
    RightMotors.setStopping(hold);
    driveDistance( -36);
  }
}
void pointRightGoalElimWall(){
  logD("RUN pointRIGHTRingsSafe", "pointRIGHTRingsSafe");

  int startTime = Brain.Timer.time(sec);

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

  task dist = task(distTaskMainLowACRight);
  turnToHeadingC(getAnglePoint(0 + xTrim, -70.8 + yTrim), "PLAIN");

  tPosLB=180;

  wait(500,msec);

  driveDistance(-6);
  setMotorVelocities(0);
  tPosLB=1;

  wait(100,msec);

  turnToHeadingC(getAnglePoint(-0 + xTrim, -47 + yTrim), "PLAIN");

  raiseIntake.set(true);

  setTargetIntake(100);

  runLower=true;

  call4DD(5.5, 4, lowerIntake);
  setMotorVelocities(0);
  
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);

  raiseIntake.set(false);

  wait(100, msec);

  //driveDistance(-4);

  //wait(100,msec);
  
  turnToHeadingC(180+getAnglePoint(24 + xTrim, -24 + yTrim), "ULTRA");

  //wait(500,sec);

  //turnToHeadingC(180+getAnglePoint(26.5 + xTrim, -24 + yTrim), "ULTRA");
  // chassis.set_turn_constants(12, 0.4897, 0.0155, 3.10, 17.2966);
  // chassis.set_turn_exit_conditions(0.8, 500, 1000);

  // chassis.turn_to_angle(180+getAnglePoint(-24 + xTrim, -24 + yTrim));
  //wait(2, sec);

  //default_constants();
  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  driveDistance(-getDistance(24, -24)+11.5);
  setMotorVelocities(0);

  
  

  //turnToHeadingC(180+getAnglePoint(-24 + xTrim, -24 + yTrim), "PLAIN");

  


  driveDistance(-8);

  call4DD(-8, 5, clampTrue);

  setTargetIntake(0);

  clampPistons.set(true);


  //wait(500,sec);


  // turnToHeadingC(getAnglePoint(1.5,-4), "ULTRA");

  // //chassis.drive_to_point(-4, 4);

  // setTargetIntake(100);
  // runTop = true;

  // driveDistance( (getDistance(1.5,-4)-10) );

  // //chassis.drive_to_point(14, -13);

  // dPistons.set(true);

  // wait(200,msec);

  // reverseLower = true;

  // driveDistance(-28);


  // reverseLower = false;
  // runTop = false;
  // runLower=true;

  // dPistons.set(false);

  // wait(50,msec);

  // turnToHeadingC(chassis.get_absolute_heading()+23);

  // driveDistance(12);








  turnToHeadingC(getAnglePoint(48, -24));

  setTargetIntake(100);
  runLower=false;

  // runLower=false;

  driveDistance(getDistance(48, -24) - 4);

  // turnToHeadingC(getAnglePoint(48, -48));

  // donaldDuck(getDistance(48, -48));

  //call4DD(getDistance(48, -48)+4, 5, revLower);
  //driveDistance(-5);
  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()), FTS(chassis.get_absolute_heading()));

  turnToHeadingC(140);

  raiseIntake.set(true);

  setDriveParams(60);

  chassis.drive_timeout=1400;
  
  //driveDistance(60);
  chassis.drive_distance(60);

  //turnToHeadingC(getAnglePoint(68, -68));

  // raiseIntake.set(true);

  // reverseLower = false;

  // setDriveParams(getDistance(65, -65));

  // chassis.drive_timeout=850;

  // chassis.drive_distance(getDistance(65, -65));

  raiseIntake.set(false);

  donaldDuck(-2);

  wait(1800,msec);

  //chassis.drive_timeout=500;

  // 

  // chassis.drive_distance(11);

  // chassis.drive_distance(-8);

  // chassis.drive_distance(11);

  // chassis.drive_distance(-8);

  // chassis.drive_distance(11);

  turnToHeadingC(245, "DOG");
  clampPistons.set(false);

  driveDistance(5);

  turnToHeadingC(180);

  if((Brain.Timer.time(sec) - startTime) < 14.1){
    LeftMotors.setStopping(hold);
    RightMotors.setStopping(hold);
    driveDistance( -36);
  }
}


//Ring Rush
void pointLeftRingRushElim(){
  
  chassis.set_coordinates(-20, -52, 334);
  chassis.desired_heading = 334;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  //vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  

  

  // clampPistons.set(true);
  // setTargetIntake(100);

  // chassis.set_drive_constants(0.3, 1, 1, 1, 1);
  // chassis.drive_timeout=100000;
  // chassis.drive_distance(1000);

  
  // wait(50,sec);
  setTargetIntake(100);
  runLower = true;

  driveDistance(45);

  driveDistance(-14);

  turnToHeadingC(270);

  wait(350,msec);

  driveDistance( - 4 );
  driveDistance( - 4 );

  call4DD(-10, 6, clampTrue);
  clampPistons.set(true);

  runLower=false;

  chassis.drive_max_voltage=4;
  chassis.drive_distance(getDistance(-48, -24) - 9);

  turnToHeadingC(342);

  // runLower=true;

  colorSort=true;

  driveDistance(14);

  driveDistance(-10);

  wait(450,msec);

  turnToHeadingC(215);

  runLower=false;

  chassis.drive_timeout=600;

  chassis.drive_distance(100);

  raiseIntake.set(true);

  chassis.drive_timeout=470;

  chassis.drive_distance(100);

  raiseIntake.set(false);

  driveDistance(-16);

  turnToHeadingC(93, "MobileGoal3");

  chassis.desired_heading = 93;
  

  driveDistance(34);

  raiseIntake.set(true);

  driveDistance(11);

  raiseIntake.set(false);

  wait(150,msec);

  driveDistance(-8);

  wait(400,msec);

  reverseLower = true;

  //driveDistance(40);

  

}
void pointRightRingRushElim(){
  chassis.set_coordinates(20, -52, -334);
  chassis.desired_heading = -334;

  //getPosDist(false, chassis.get_absolute_heading(), 5, 15);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);
 tPosLB = 1;
 needReset = true;
  
  //tPosLB = 165;
  //teamColor="RED";
  

  

  // clampPistons.set(true);
  // setTargetIntake(100);

  // chassis.set_drive_constants(0.3, 1, 1, 1, 1);
  // chassis.drive_timeout=100000;
  // chassis.drive_distance(1000);

  
  // wait(50,sec);
  setTargetIntake(100);
  runLower = true;

  driveDistance(45);

  driveDistance(-13);

  turnToHeadingC(-270);

  wait(350,msec);

  driveDistance( - 6 );
  wait(100,msec);
  driveDistance( - 6 );
  wait(100,msec);

  call4DD(-10, 6, clampTrue);
  clampPistons.set(true);

  runLower=false;

  chassis.drive_max_voltage=4;
  chassis.drive_distance(getDistance(48, -24) - 9);

  turnToHeadingC(-342);

  // runLower=true;

  colorSort=true;

  driveDistance(13);

  driveDistance(-10);

  wait(450,msec);

  turnToHeadingC(-215);

  runLower=false;

  chassis.drive_timeout=600;

  chassis.drive_distance(100);

  raiseIntake.set(true);

  chassis.drive_timeout=530;

  chassis.drive_distance(100);

  raiseIntake.set(false);

  driveDistance(-16);

  turnToHeadingC(-93, "MobileGoal3");

  chassis.desired_heading = 360-93;
  

  driveDistance(34);

  //Touch
    //tPosLB=160;
  //turnToHeadingC(-20);
  //   setMotorVelocities(100);
  // wait(400,msec);
  // setMotorVelocities(00);


  raiseIntake.set(true);

  driveDistance(8);

  raiseIntake.set(false);

  wait(150,msec);

  driveDistance(-14);

  wait(400,msec);
  
  driveDistance(40);
}



//RED, alliance stake, 4 rings on mogoal AWP
void pointLeftRingsSafe(){
  bool safe = true;
  logD("RUN pointLeftRingsSafe", "pointLeftRingsSafe");

  //setup();

  odom_constants();

  float xTrim = -1;
  float yTrim = 0;

  chassis.set_coordinates(-12, -60, 90);
  //chassis.desired_heading = 90;

  getPosDist(false, chassis.get_absolute_heading(), 50, 6);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  colorSort=true;

  task dist = task(distTaskMainLowAC);

  if(safe){
    //wait(2,seconds);
  }

  //turnToHeadingC(getAnglePoint(0 + xTrim, -70.8 + yTrim), "PLAIN");
  turnToHeadingC(135);
  tPosLB=180;

  speedLB = 0.35;


  if(safe){

    //wait(500,msec);

  }

  wait(1000,msec);

  if(safe){
    
  }
  speedLB = 1;

  driveDistance(-6);
  setMotorVelocities(0);
  tPosLB=1;

  wait(100,msec);

  turnToHeadingC(getAnglePoint(-0 + xTrim, -47 + yTrim), "PLAIN");

  raiseIntake.set(true);

  setTargetIntake(100);

  runLower=true;

  call4DD(8, 5, lowerIntake);
  setMotorVelocities(0);
  
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);

  raiseIntake.set(false);

  wait(250,msec);

  driveDistance(-4);

  wait(100,msec);
  
  turnToHeadingC(180+getAnglePoint(-25.5 + xTrim, -24 + yTrim), "ULTRA");
  // chassis.set_turn_constants(12, 0.4897, 0.0155, 3.10, 17.2966);
  // chassis.set_turn_exit_conditions(0.8, 500, 1000);

  // chassis.turn_to_angle(180+getAnglePoint(-24 + xTrim, -24 + yTrim));
  //wait(2, sec);

  //default_constants();
  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  driveDistance(-getDistance(-25.5, -24)+11.5);
  setMotorVelocities(0);

  
  

  //turnToHeadingC(180+getAnglePoint(-24 + xTrim, -24 + yTrim), "PLAIN");

  colorSort=true;


  driveDistance(-6);

  call4DD(-8, 4, clampTrue);

  setTargetIntake(0);

  setTargetIntake(50);
  runLower=false;

  clampPistons.set(true);

  wait(50,msec);

  //turnToHeadingC(getAnglePoint(-48 + xTrim, -24 + yTrim), "MobileGoal0");
   //turnToHeadingC(getAnglePoint(-48 + xTrim, -24 + yTrim), "MobileGoal0");
  turnToHeadingC(262);

  setTargetIntake(100);
  runLower=false;

  chassis.drive_max_voltage = 10;

  //driveDistance(getDistance(-48, -24) - 4);
  driveDistance(20);
  //chassis.drive_to_point(-48, -24);

  if(safe){
   
    runLower = false;
    wait(2, seconds);
     tPosLB=140;
    
    turnToHeadingC(83);
    

    LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);

  setMotorVelocities(100);
  wait(450,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);

    wait(50,sec);
  }

  setMotorVelocities(0);

  wait(250,msec);

  turnToHeadingC(getAnglePoint(-48 + xTrim, -48 + yTrim), "MobileGoal3");

  chassis.set_drive_constants(10, 1.58205, 0.02075, 9.7, 1.19);
  chassis.drive_distance(getDistance(-48 + xTrim, -48 + yTrim)-3);
  //chassis.drive_to_point(-48, -48);

  turnToHeadingC(getAnglePoint(-65 + xTrim, -65 + yTrim), "MobileGoal3");

  raiseIntake.set(true);
  

  setDriveParams(getDistance(-65, -66)-2);
  chassis.drive_timeout=800;

  chassis.drive_distance(getDistance(-65, -66)+2);
  raiseIntake.set(false);
  wait(200,msec);
  driveDistance(-2);
  //wait(200,msec);
  wait(1300,msec);
  // setMotorVelocities(20);
  // wait(300,msec);
  // setMotorVelocities(00);
  // wait(200,msec);

  // setMotorVelocities(-10);
  // wait(250,msec);

  turnToHeadingC(45, "MobileGoal3");
  setTargetIntake(0);
  tPosLB=140;

  LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);

  setMotorVelocities(100);
  wait(650,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);
}





void pointRightRingsSafe(){
  bool safe = false;
  bool elims = true;


  logD("RUN pointRIGHTRingsSafe", "pointRIGHTRingsSafe");

  setup();

  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(12, -60, 270);
  //chassis.desired_heading = 90;

  getPosDist(true, chassis.get_absolute_heading(), 50, 10);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  //wait(5,sec);
  //tPosLB = 165;
  //teamColor="RED";
  colorSort=true;

  task dist = task(distTaskMainLowACRight);
  if(safe){
    //wait(4, seconds);
  }

  turnToHeadingC(-135);//getAnglePoint(0 + xTrim, -70.8 + yTrim), "PLAIN");

  tPosLB=180;
  speedLB = 0.35;

  wait(1200,msec);
  speedLB = 1;

  driveDistance(-6);
  setMotorVelocities(0);
  tPosLB=1;

  wait(100,msec);

  turnToHeadingC(getAnglePoint(-0 + xTrim, -44 + yTrim), "PLAIN");

  raiseIntake.set(true);

  setTargetIntake(100);

  runLower=true;

  call4DD(8, 5, lowerIntake);
  setMotorVelocities(0);
  
  LeftMotors.setStopping(brake);
  RightMotors.setStopping(brake);

  raiseIntake.set(false);

  wait(250,msec);

  driveDistance(-4);

  wait(100,msec);

  //wait(500,sec);
  
  turnToHeadingC(180+getAnglePoint(26.5 + xTrim, -24 + yTrim), "ULTRA");

  turnToHeadingC(180+getAnglePoint(26.5 + xTrim, -24 + yTrim), "ULTRA");
  // chassis.set_turn_constants(12, 0.4897, 0.0155, 3.10, 17.2966);
  // chassis.set_turn_exit_conditions(0.8, 500, 1000);

  // chassis.turn_to_angle(180+getAnglePoint(-24 + xTrim, -24 + yTrim));
  //wait(2, sec);

  //default_constants();
  //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
  driveDistance(-getDistance(26.5, -24)+11.5);
  setMotorVelocities(0);

  colorSort=true;
  

  //turnToHeadingC(180+getAnglePoint(-24 + xTrim, -24 + yTrim), "PLAIN");

  


  driveDistance(-6);

  call4DD(-8, 4, clampTrue);

  setTargetIntake(0);

  setTargetIntake(60);
  runLower=false;

  clampPistons.set(true);

  wait(50,msec);

  // turnToHeadingC(getAnglePoint(45 + xTrim, -24 + yTrim), "MobileGoal0");
  turnToHeadingC(90);

  setTargetIntake(100);
  runLower=false;

  // chassis.drive_max_voltage = 10;

  // //driveDistance(getDistance(-48, -24));
  driveDistance(24);
  // chassis.drive_to_point(44, -24);

  // setMotorVelocities(0);

  // wait(1000,msec);

  // chassis.turn_to_angle(0);

  if(safe){
    runLower = false;
    wait(2, seconds);
    tPosLB = 140;
    turnToHeadingC(270);
    

    LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);

  setMotorVelocities(100);
  wait(650,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);

    wait(50,sec);
  }
 
  bool clearCornor = false;
  if(clearCornor){
    turnToHeadingC(180);
    setMotorVelocities(100);
    wait(400,msec);
    setTargetIntake(0);
    wait(400,msec);
    setMotorVelocities(40);
    wait(800,msec);
    turnToHeadingC(100);
    dPistons.set(true);
    setMotorVelocities(30);
    wait(800,msec);
    turnToHeadingC(-45);
    tPosLB = 140;
    dPistons.set(false);
    LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);

  setMotorVelocities(100);
  wait(650,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);

  wait(50,sec);
  }


  turnToHeadingC(getAnglePoint(48 + xTrim, -48 + yTrim), "MobileGoal3");

  chassis.set_drive_constants(10, 1.58205, 0.02075, 9.7, 1.19);
  chassis.drive_distance(getDistance(48 + xTrim, -48 + yTrim)-3);
  //chassis.drive_to_point(-48, -48);

  

  turnToHeadingC(getAnglePoint(65 + xTrim, -69 + yTrim), "MobileGoal3");

  

  raiseIntake.set(true);
  

  setDriveParams(getDistance(65, -68)-2);
  chassis.drive_timeout=800;

  

  chassis.drive_distance(getDistance(65, -69));
  raiseIntake.set(false);
  wait(200,msec);
  driveDistance(-2);
 
  if(elims){
    wait(300,msec);
    driveDistance(-4);
    wait(300,msec);
    setMotorVelocities(30);
    wait(400, msec);
    driveDistance(-4);
    setMotorVelocities(30);
    wait(400, msec);
  }
   wait(1500,msec);
  // wait(300,msec);
  // setMotorVelocities(100);
  // wait(400,msec);
  // driveDistance(-4);

  if(elims){
    wait(400,msec);
    driveDistance(3);
    wait(100,sec);
  }
  // wait(00,msec);
  // setMotorVelocities(20);
  // wait(300,msec);
  // setMotorVelocities(00);
  // wait(200,msec);

  // setMotorVelocities(-10);
  // wait(250,msec);

  tPosLB=140;

  turnToHeadingC(-50, "MobileGoal3");
  //setTargetIntake(0);
  

  LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);

  setMotorVelocities(100);
  wait(650,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);
}







void pointSkillsOLD(){
  logD("RUN pointSkills", "pointSkills");


  odom_constants();

  float xTrim = 0;
  float yTrim = 0;

  chassis.set_coordinates(-0, -60, 00);
  chassis.desired_heading = 00;

  //getPosDist(false, chassis.get_absolute_heading(), 50, 8);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  //colorSort=true;

  task dist = task(distTaskMainLowAC);

  //call4DD(-14, 10, clampTrue);

  //turnToHeadingC(getAnglePoint(-24, -24));
  
  setTargetIntake(100);
  // colorSort=true;
  // teamColor="RED";


  tPosLB=1;


  wait(350,msec);
  driveDistance(8);
  
  //turnToHeadingC(180 + getAnglePoint(-24, -48));
  turnToHeadingC(90);

  driveDistance(-18);// -(getDistance(-24, -48) - 10) );

  call4DD(-12, 8, clampTrue);

  turnToHeadingC(270);

  driveDistance(24);

  wait(1500,msec);

  turnToHeadingC(45);
  
  setMotorVelocities(-100);

  wait(600,msec);
  setMotorVelocities(0);

  clampPistons.set(false);

  wait(200,msec);

  driveDistance(8);

  turnToHeadingC(180 + 90);

  driveDistance(-80);

  driveDistance(-16);
  //driveDistance(-6);
  //driveDistance(-6);

  turnToHeadingC(-45);

  setMotorVelocities(-100);
  wait(800,msec);

  setMotorVelocities(0);

  clampPistons.set(false);

  driveDistance(10);

}




void pointSkillsJAKE() {
    

    odom_constants();  // Ensure odometry constants are set

    float xTrim = 0;
    float yTrim = 0;

    // Set the initial robot position
    chassis.set_coordinates(0, -60, 0);
    chassis.desired_heading = 0;

    vex::task earthIntake = vex::task(runIntake);  // Start intake
    vex::task flatEarth = vex::task(spinToPosTask);
   
  tPosLB = 80;
  wait(500,msec);

    // Move forward and run the intake
    setTargetIntake(100);
    wait(1000, msec);
    setTargetIntake(0);

      tPosLB = 1;
      wait(500,msec);

    driveDistance(13);
    turnToHeadingC(-90);
    LeftMotors.setVelocity(20, percent);
    RightMotors.setVelocity(20, percent);
    driveDistance(-18);
    call4DD(-7, 4.5, clampTrue);
    clampPistons.set(true);

    wait(100,msec);
    LeftMotors.setVelocity(100, percent);
    RightMotors.setVelocity(100, percent);

    turnToHeadingC(5);
    setTargetIntake(100);
    driveDistance(20);
    turnToHeadingC(50);
    driveDistance(12);
    turnToHeadingC(15);


    LeftMotors.setVelocity(10, percent);
    RightMotors.setVelocity(10, percent);  
    tPosLB = 360-335;
    //raise lady brown to loading position here
    driveDistance(55);
    wait(1000,msec);
    driveDistance(-34);
    turnToHeadingC(90);
    wait(1600,msec);
    setTargetIntake(0);
    wait(400,msec);

    LeftMotors.setVelocity(10, percent);
    RightMotors.setVelocity(10, percent);  
    tPosLB = 90;
    setTargetIntake(-10);
    //raise lady brwon to neutral position
    wait(500,msec);
    setTargetIntake(100);
    driveDistance(26);
    tPosLB = 170;
    //lower lady brown to score
    driveDistance(-14);
    tPosLB = 1;
    //return lady brown to lowest position
    turnToHeadingC(180);
    
    driveDistance(68);
    
    wait(1000,msec);
    turnToHeadingC(45);
    driveDistance(21);
    turnToHeadingC(-30);
    driveDistance(-34);
    clampPistons.set(false);
    
    setTargetIntake(100);
    driveDistance(7);

    wait(50,sec);
    turnToHeadingC(-20);
    //only run bottom motor
    driveDistance(100);
    setTargetIntake(0);
    turnToHeadingC(109);
    driveDistance(-22);
    clampPistons.set(true);











    



    
}



void pointSkillsBackup(){
  logD("RUN pointSkills", "pointSkills");


 

  chassis.desired_heading = 0;

  //getPosDist(false, chassis.get_absolute_heading(), 50, 8);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  //colorSort=true;



  //call4DD(-14, 10, clampTrue);

  //turnToHeadingC(getAnglePoint(-24, -24));
  
  setTargetIntake(100);
  // colorSort=true;
  // teamColor="RED";

  tPosLB=1;
  needReset = true;

  
  chassis.set_heading(0);

  wait(350,msec); 
  driveDistance(14);
  
  
  //turnToHeadingC(180 + getAnglePoint(-24, -48));
  turnToHeadingC(-90);
   //wait(150,msec); //350
  chassis.drive_distance(-13);
  chassis.drive_distance(-8);
  clampPistons.set(true);
  wait(0.2,seconds); // tune if needed 
  turnToHeadingC(0);
  setTargetIntake(100);
  chassis.drive_distance(20);
  turnToHeadingC(29);
  tPosLB = 360-335;
  chassis.drive_distance(52);
  wait(0.25,seconds); //0.25

  turnToHeadingC(10);
  
  chassis.drive_distance(-16);

  setTargetIntake(-10);
  tPosLB = 60;

  turnToHeadingC(90);

  setTargetIntake(50);
  
  chassis.drive_timeout = 900;
  chassis.drive_distance(17);//17
  tPosLB = 180;
  wait(0.2,seconds);
  
  setTargetIntake(100);
  wait(0.2,seconds);
  chassis.drive_timeout = 600;
  chassis.drive_distance(17);

  chassis.drive_timeout = 3000;

 //360-210;
  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4);
  
  chassis.drive_distance(-14);
  tPosLB=1;
  wait(0.4,seconds);

  turnToHeadingC(177);
  setTargetIntake(100);

  chassis.drive_distance(28);
  wait(350,msec);
  chassis.drive_distance(20);
    wait(350,msec);
  chassis.drive_distance(10);
    wait(350,msec); //500
  turnToHeadingC(75);
  
  chassis.drive_distance(7);
  
  turnToHeadingC(335);
  chassis.drive_timeout = 800;
  chassis.drive_distance(-11);
  clampPistons.set(false);
  wait(50,msec);
  chassis.drive_timeout = 3000;
  //wait(400,msec);
  chassis.drive_distance(10.5);
  turnToHeadingC(90);
  getPosDist(true, chassis.get_absolute_heading()+2, 30, 100000);
  chassis.desired_heading = 180+getAnglePoint(-24, -48);
  chassis.drive_distance(-37);
  chassis.desired_heading = 180+getAnglePoint(-24, -48);
  chassis.drive_distance(-(getDistance(-24, -48) - 12));
  //wait(0.25,seconds); //0.5
  //chassis.drive_distance(-9);// changed from 9
  call4DD(-12, 8, clampTrue);
  clampPistons.set(true);

  //
  //
  turnToHeadingC(0);
  setTargetIntake(100);
  chassis.drive_distance(20);
  turnToHeadingC(336);
  
  call4DD(52, 30, loadLB);
  //chassis.drive_distance(52);
  wait(0.5,seconds);
  turnToHeadingC(350);
  chassis.drive_distance(-20.5); //tune rnrn
  
  setTargetIntake(-10);//raise 2
  tPosLB = 60;

  turnToHeadingC(270);
  
  setTargetIntake(50);

  //chassis.drive_timeout = 900;
  //chassis.drive_distance(18); // too much
  setMotorVelocities(60);
  wait(800,msec);
  tPosLB = 180;
  wait(0.2,seconds);
  setTargetIntake(100);
  wait(0.2,seconds);
  chassis.drive_timeout = 600;
  //chassis.desired_heading = chassis.get_absolute_heading();
  chassis.drive_distance(18);

  chassis.drive_timeout = 3000;

  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4.4);
  //
  //chassis.desired_heading = chassis.get_absolute_heading();
  chassis.drive_distance(-12); // tune if necessary
  tPosLB=1;
  needReset = true;
  wait(0.4,seconds);

  turnToHeadingC(180);
  needReset = true;
  setTargetIntake(100);
  chassis.drive_distance(28);
  wait(350,msec);
  chassis.drive_distance(20);
    wait(350,msec);
  chassis.drive_distance(10);
    wait(750,msec); //850
    turnToHeadingC(300);
    chassis.drive_distance(11);
    turnToHeadingC(45);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(-11);
  clampPistons.set(false);
  wait(50,msec);
  chassis.drive_timeout = 3000;
  chassis.drive_distance(15);

  //Second Half 1 top ring 2 blue goals in corner 

    turnToHeadingC(0);
    chassis.drive_distance(23);
    //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
    //wait(500,msec);
    getPosDist(false, chassis.get_absolute_heading()+2, 50, 100000);
    //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
    chassis.drive_distance(40);
    turnToHeadingC(getAnglePoint(-23, 23));
    chassis.drive_distance(getDistance(-23, 23) - 2);
      setTargetIntake(100);
      wait(0.1, sec);
      setTargetIntake(0);
      turnToHeadingC(180+getAnglePoint(3, 47));





      chassis.drive_distance(-26);
      chassis.drive_distance(-(getDistance(0, 47) - 12)); 
      call4DD(-12, 9, clampTrue);
      clampPistons.set(true);
      wait(350,msec);
      setTargetIntake(100);
      wait(0.5, sec); // tune 
      turnToHeadingC(45);
      chassis.drive_distance(-15);
      clampPistons.set(false);
      chassis.drive_distance(12);
      turnToHeadingC(75);

      setTargetIntake(-100);
      chassis.drive_timeout = 2000;
      chassis.drive_distance(58); //changed from 54 
      chassis.drive_timeout = 3000;

      chassis.drive_distance(-27); 
      turnToHeadingC(97.5);
      chassis.drive_timeout = 2000;
      chassis.drive_distance(-105); //changed from 115 
      chassis.drive_distance(10);
       turnToHeadingC(330); //tune if needed
       tPosLB=130;
       setMotorVelocities(-100);
       wait(800,msec); 
       setMotorVelocities(-40);

       wait(800,msec);
       tPosLB = 1;
       setMotorVelocities(50);


}

void pointSkills(){
  logD("RUN pointSkills", "pointSkills");


  odom_constants();

  chassis.desired_heading = 0;

  //getPosDist(false, chassis.get_absolute_heading(), 50, 8);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  //colorSort=true;



  //call4DD(-14, 10, clampTrue);

  //turnToHeadingC(getAnglePoint(-24, -24));
  
  setTargetIntake(100);
  // colorSort=true;
  // teamColor="RED";

  tPosLB=1;
  needReset = true;

  
  chassis.set_heading(0);

  wait(350,msec); 
  driveDistance(14);
  
  
  //turnToHeadingC(180 + getAnglePoint(-24, -48));
  turnToHeadingC(-90);
   //wait(150,msec); //350
  driveDistance(-13);
  driveDistance(-8);
  clampPistons.set(true);
  wait(0.2,seconds); // tune if needed 
  turnToHeadingC(0);
  setTargetIntake(100);
  driveDistance(20);
  turnToHeadingC(29);
  
  //call4DD(52, 30, loadLB);
  driveDistance(52);
  loadLB();
  wait(0.25,seconds); //0.25

  turnToHeadingC(10);
  
  driveDistance(-16);

  //setTargetIntake(-10);
  //tPosLB = 60;

  turnToHeadingC(90);

  setTargetIntake(50);

  chassis.drive_timeout = 300;
  chassis.drive_distance(15);
  
  setMotorVelocities(60);
  wait(200,msec);

  tPosLB = 180;
  setTargetIntake(-35);
  wait(0.2,seconds);
  setTargetIntake(100);

  wait(0.3,seconds);
  
  setTargetIntake(100);
  wait(0.2,seconds);
  driveDistance(-3);
  chassis.drive_timeout = 600;
  chassis.drive_distance(17);

  chassis.drive_timeout = 3000;

 //360-210;
  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4);
  
  driveDistance(-14);
  tPosLB=1;
  wait(0.4,seconds);

  turnToHeadingC(177);
  setTargetIntake(100);

  driveDistance(28);
  wait(350,msec);
  driveDistance(20);
    wait(350,msec);
  driveDistance(10);
    wait(350,msec); //500
  turnToHeadingC(75);
  
  driveDistance(7);
  
  turnToHeadingC(335);
  chassis.drive_timeout = 800;
  chassis.drive_distance(-11);
  clampPistons.set(false);
  wait(50,msec);
  chassis.drive_timeout = 3000;
  //wait(400,msec);
  driveDistance(10.5);
  turnToHeadingC(90);
  getPosDist(true, chassis.get_absolute_heading()+2, 30, 10000);
  chassis.desired_heading = 180+getAnglePoint(-24, -48);
  driveDistance(-37);
  chassis.desired_heading = 180+getAnglePoint(-24, -48);
  driveDistance(-(getDistance(-24, -48) - 14));
  driveDistance(-8);
  //wait(0.25,seconds); //0.5
  //chassis.drive_distance(-9);// changed from 9
  call4DD(-10, 6, clampTrue);
  clampPistons.set(true);

  //
  //
  turnToHeadingC(0);
  setTargetIntake(100);
  driveDistance(20);
  turnToHeadingC(336);
  
  //call4DD(52, 30, loadLB);
  driveDistance(52);
  loadLB();
  //chassis.drive_distance(52);
  wait(0.5,seconds);
  turnToHeadingC(350);
  driveDistance(-20.5); //tune rnrn

  // setTargetIntake(-10);//raise 2
  // tPosLB = 60;

  turnToHeadingC(270);
  
  setTargetIntake(50);

  //chassis.drive_timeout = 900;
  //chassis.drive_distance(18); // too much

  chassis.drive_timeout = 300;
  chassis.drive_distance(15);

  setMotorVelocities(60);
  wait(200,msec);
  
  tPosLB = 180;
  setTargetIntake(-35);
  wait(0.2,seconds);
  setTargetIntake(100);
  wait(0.4,seconds);
  driveDistance(-3);
  chassis.drive_timeout = 600;
  //chassis.desired_heading = chassis.get_absolute_heading();
  chassis.drive_distance(18);

  chassis.drive_timeout = 3000;

  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4.4);
  //
  //chassis.desired_heading = chassis.get_absolute_heading();
  driveDistance(-12); // tune if necessary
  tPosLB=1;
  needReset = true;
  wait(0.4,seconds);

  turnToHeadingC(180);
  needReset = true;
  setTargetIntake(100);
  driveDistance(28);
  wait(350,msec);
  driveDistance(20);
    wait(350,msec);
  driveDistance(10);
    wait(750,msec); //850
    turnToHeadingC(300);
    driveDistance(11);
    turnToHeadingC(45);
    chassis.drive_timeout = 1000;
    chassis.drive_distance(-11);
  clampPistons.set(false);
  wait(50,msec);
  chassis.drive_timeout = 3000;
  driveDistance(15);

  //Second Half 1 top ring 2 blue goals in corner 

    turnToHeadingC(0);
    driveDistance(23);
    //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
    //wait(500,msec);
    getPosDist(false, chassis.get_absolute_heading()+2, 50, 30);
    //task dist = task(distTaskMainLowAC);
    
    //displayInfo(Controller1, FTS(chassis.get_X_position()), FTS(chassis.get_Y_position()));
    driveDistance(20);
    //turnToHeadingC(getAnglePoint(-23, 23));
    turnToHeadingC(47);
    displayInfo(Controller1, FTS(getDistance(-23, 23)));
    // if(getDistance(-23, 23)>45 && getDistance(-23, 23)<55){
    //   turnToHeadingC(getDistance(-23, 23));
    // }
    // else{
    // turnToHeadingC(50);
    // }


    // driveDistance(getDistance(-23, 23) - 2);
    //   setTargetIntake(100);
    //   wait(0.1, sec);
    //   setTargetIntake(0);
    //   //dist = nullptr;
    //   turnToHeadingC(180+getAnglePoint(3, 47));





    //   driveDistance(-26);

    //   default_constants(); //reset driveconstants because DriveDistance modifies them
    //   chassis.drive_distance(-(getDistance(3, 47) - 12));
    //   //driveDistance(); 
    //   call4DD(-12, 9, clampTrue);
    //   clampPistons.set(true);
    //   wait(350,msec);
    //   setTargetIntake(100);
    //   wait(0.5, sec); // tune 
    //   turnToHeadingC(45);
    //   driveDistance(-15);
    //   clampPistons.set(false);
    //   driveDistance(12);
    //   turnToHeadingC(75);

    setTargetIntake(100);
    runLower = true;

    chassis.drive_distance(38);
      
    turnToHeadingC(225);





      chassis.drive_distance(-16);
      //wait(2,seconds);
      chassis.drive_distance(-8); 
      call4DD(-8, 5, clampTrue);
      clampPistons.set(true);

      tPosLB = 1;
      needReset = true;

      wait(50,msec);
      runLower = false;
      setTargetIntake(100);

      wait(0.6, sec); // tune 
      turnToHeadingC(45);
      chassis.drive_distance(-10);
      
      clampPistons.set(false);
      wait(0.05, sec);
      chassis.drive_distance(10);



      turnToHeadingC(75+180);
      chassis.drive_timeout = 2000;
      chassis.drive_distance(-65);
      chassis.drive_timeout = 3000;
      chassis.drive_distance(45); 

      // turnToHeadingC(75);
      // runLower = false;
      // setTargetIntake(-100);
      // chassis.drive_timeout = 2000;
      // chassis.drive_distance(65); //changed from 54 
      // chassis.drive_timeout = 3000;
      //chassis.drive_distance(-45); 



      turnToHeadingC(110);
      chassis.drive_timeout = 2000;
      chassis.drive_distance(-80); //changed from 015 
      chassis.drive_distance(10);
       turnToHeadingC(320); //tune if needed
       tPosLB=130;
       setMotorVelocities(-100);
       wait(650,msec); 
       
       
       LeftMotors.stop(coast);
      RightMotors.stop(coast);

       wait(500,msec); 
      

      //if(Inertial.)
       wait(800,msec);
       tPosLB = 1;
       setMotorVelocities(40);
       //wait(800,msec);


}

void pointSkillsAKS(){
  logD("RUN pointSkills", "pointSkills");


 

  chassis.desired_heading = 0;

  //getPosDist(false, chassis.get_absolute_heading(), 50, 8);

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  //tPosLB = 165;
  //teamColor="RED";
  //colorSort=true;



  //call4DD(-14, 10, clampTrue);

  //turnToHeadingC(getAnglePoint(-24, -24));
  
  setTargetIntake(100);
  // colorSort=true;
  // teamColor="RED";


  tPosLB=1;

  
  chassis.set_heading(0);

  wait(350,msec); 
  driveDistance(14);
  
  
  //turnToHeadingC(180 + getAnglePoint(-24, -48));
  turnToHeadingC(-90);
   //wait(150,msec); //350
  chassis.drive_distance(-13);
  chassis.drive_distance(-8);
  clampPistons.set(true);
  wait(0.2,seconds); // tune if needed 
  turnToHeadingC(0);
  setTargetIntake(100);
  chassis.drive_distance(20);
  turnToHeadingC(29);
  chassis.drive_distance(52);
  wait(0.15,seconds); //0.25

  turnToHeadingC(10);
  chassis.drive_distance(-16);
  turnToHeadingC(90);
  tPosLB = 360-335;
  chassis.drive_distance(17);//17
  wait(1,seconds);
  setTargetIntake(0);

  tPosLB = 360-210;
  setTargetIntake(-10);
  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4);
  wait(0.5,seconds);
  chassis.drive_distance(-14);
  tPosLB=1;
  turnToHeadingC(180);
  setTargetIntake(100);
  chassis.drive_distance(28);
  wait(350,msec);
  chassis.drive_distance(20);
    wait(350,msec);
  chassis.drive_distance(10);
    wait(350,msec); //500
  turnToHeadingC(75);
  
  chassis.drive_distance(10);
  
  turnToHeadingC(335);
  chassis.drive_distance(-11);
  clampPistons.set(false);
  //wait(400,msec);
  chassis.drive_distance(10.5);
  turnToHeadingC(90);
  chassis.drive_distance(-37);
  chassis.drive_distance(-32);
  //wait(0.25,seconds); //0.5
  chassis.drive_distance(-9);// changed from 9
  clampPistons.set(true);

  //
  turnToHeadingC(0);
  setTargetIntake(100);
  chassis.drive_distance(20);
  turnToHeadingC(336);
  chassis.drive_distance(52);
  wait(0.5,seconds);
  turnToHeadingC(350);
  chassis.drive_distance(-20.5); //tune rnrn
  turnToHeadingC(270);
  tPosLB = 360-335;
  chassis.drive_distance(18); // too much
  wait(0.5,seconds);
  setTargetIntake(0);

  tPosLB = 360-210;
  setTargetIntake(-10);
  //chassis.drive_distance(-3.25);
  //chassis.drive_distance(4.4);
  wait(0.2,seconds);
  chassis.drive_distance(-12); // tune if necessary
  tPosLB=1;
  turnToHeadingC(180);
  setTargetIntake(100);
  chassis.drive_distance(28);
  wait(350,msec);
  chassis.drive_distance(20);
    wait(350,msec);
  chassis.drive_distance(10);
    wait(750,msec); //850
    turnToHeadingC(300);
    chassis.drive_distance(11);
    turnToHeadingC(45);
    chassis.drive_distance(-11);
  clampPistons.set(false);

  //Second Half 1 top ring 2 blue goals in corner 

    turnToHeadingC(15);
    chassis.drive_distance(35);
    chassis.drive_distance(20);
    turnToHeadingC(45);
    chassis.drive_distance(34);
      setTargetIntake(100);
      wait(0.1, sec);
      setTargetIntake(0);
      turnToHeadingC(225);





      chassis.drive_distance(-26);
      chassis.drive_distance(-13); 
      clampPistons.set(true);
      wait(350,msec);
      setTargetIntake(100);
      wait(0.25, sec); // tune 
      turnToHeadingC(45);
      chassis.drive_distance(-15);
      wait(0.25, sec);
      clampPistons.set(false);
      chassis.drive_distance(10);
      turnToHeadingC(75);
      
      setTargetIntake(-100);
      chassis.drive_distance(58); //changed from 54 
      chassis.drive_distance(-27); 
      turnToHeadingC(100);
      chassis.drive_distance(-105); //changed from 115 
      chassis.drive_distance(20);
       turnToHeadingC(355); //tune if needed
       chassis.drive_distance(-20);
}


void sawp(){
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

  task dist = task(distTaskMainLowACRight);
}