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

#include <string>
#include <random>
#include <array>





// void autonOne(){
//   logD("RUN AUTON1", "Running auton 1");

//   setup();

//   odom_constants();

//   chassis.set_heading(180);
//   chassis.desired_heading=180;

//   driveDistance(-22);//-35);
//   driveDistance(-6);
//   driveDistance(-6);
//   driveDistance(-4);

//   clampPistons.set(true);
  
//   turnToHeadingC(260);

//   Intake.setVelocity(100, pct);
  
//   driveDistance(20);
//   wait(15,msec);
//   driveDistance(12);

//   wait(150,msec);

//   Intake.setVelocity(0, pct);

//   turnToHeadingC(-7);

//   Intake.setVelocity(100, pct);

//   wait(550,msec);

//   Intake.setVelocity(100, pct);

//   driveDistance(13);
  
//   driveDistance(4);

//   wait(200,msec);

//   driveDistance(-12);

//   turnToHeadingC(90);
//   driveDistance(5);
//   turnToHeadingC(0);

//   driveDistance(13.5);
//   driveDistance(4);
//   wait(300,msec);
//   driveDistance(-1);

//   wait(1000,msec);
  
//   Intake.setVelocity(100, pct);

//   wait(400,msec);
//   setMotorVelocities(-100);
//   wait(400,msec);
//   setMotorVelocities(100);
//   wait(200,msec);
//   setMotorVelocities(0);

//   vex::task flatEarth = vex::task(spinToPosTask);

//   turnToHeadingC((360-253-110)+60);
//   tPosLB=80;

//   setMotorVelocities(30);
//   wait(2000,msec);
//   setMotorVelocities(0);
  
// }


// void autonTwo(){

  
  
//   logD("RUN AUTON2", "Running auton 2");
  
//   setup();

//   odom_constants();

//   chassis.set_heading(180);
//   chassis.desired_heading=180;
//   vex::task flatEarth = vex::task(spinToPosTask);

//   speedLB=1;
//   tPosLB = 360-35;

//   driveDistance(-22);
//   driveDistance(-6);
//   driveDistance(-4);

//   clampPistons.set(true);
  
//   turnToHeadingC(-270);
//   speedLB=1;

//   Intake.setVelocity(100, pct);
  
//   driveDistance(26.5);
//   wait(15,msec);
//   driveDistance(3);

//   wait(150,msec);

//   Intake.setVelocity(100, pct);

//   turnToHeadingC(7);

//   Intake.setVelocity(100, pct);

//   wait(550,msec);

//   Intake.setVelocity(100, pct);

//   driveDistance(13);
  
//   driveDistance(4);

//   wait(200,msec);

//   driveDistance(-12);

//   turnToHeadingC(-20);
  

//   driveDistance(8);
//   driveDistance(5);
//   wait(300,msec);
//   driveDistance(-4);
//   turnToHeadingC(236.8);
//   driveDistance(24);
  
//   raiseIntake.set(true);
//   driveDistance(24);
//   wait(200,msec);
//   raiseIntake.set(false);
//   driveDistance(7);
//   wait(2,seconds);
//   Intake.setVelocity(100, pct);

//   turnToHeadingC(180);
//   setMotorVelocities(20);
//   wait(500,msec);
//   tPosLB = 360-200;
//   wait(500,msec);
//    tPosLB = 1;
//   driveDistance(-10);

//    /*
//    clampPistons.set(false);
//   raiseIntake.set(true);
//   wait(200,msec);
//   driveDistance(22);
//   wait(200,msec);
//   raiseIntake.set(false);
//   driveDistance(5);
//   wait(2,seconds);
//   driveDistance(5);
//   driveDistance(-8);
  
//   wait(0.8,seconds);
//   Intake.setVelocity(0,pct);
//   turnToHeadingC(-13-90-22-14);
//   driveDistance(18);
//   turnToHeadingC(-260+180);
//   driveDistance(-7.5);
//   turnToHeadingC(-260+180+80);

//   chassis.set_heading(0);
//   chassis.drive_distance(-5,0,20,20);
//   Intake.setVelocity(80,pct);
 
//   */
  
  
  
  

  



  


 
// }


void autonOne(){

  
  
  logD("RUN AUTON2", "Running auton 2");
  
  setup();

  odom_constants();

  chassis.set_heading(180);
  chassis.desired_heading=180;

  vex::task flatEarth = vex::task(spinToPosTask);
  speedLB=0.7;
  tPosLB = 360-346;


  driveDistance(-22);//-35);
  driveDistance(-6);
  driveDistance(-4);

  clampPistons.set(true);
  wait(40,msec);
  
  turnToHeadingC(270);
  speedLB=1;

  Intake.setVelocity(100, pct);
  
  driveDistance(13);
  wait(15,msec);
  driveDistance(4);

  wait(350,msec);


  turnToHeadingC(-0);


  driveDistance(13);
  
  driveDistance(3);

  wait(140,msec);

  driveDistance(-12);

  //driveDistance(-18);
  //driveDistance(6);

  //turnToHeadingC(-13);
  turnToHeadingC(-28);
  
  //driveDistance(9);
  driveDistance(9);
  driveDistance(4);
  wait(40,msec);
  driveDistance(-4);
  turnToHeadingC(-214);

  

  driveDistance(50);

  Intake.setVelocity(-100, percent);

  turnToHeadingC(-270);

  driveDistance(26);

  driveDistance(-5.5-3);
  
  
 turnToHeadingC(-180);

 setMotorVelocities(30);

 wait(350,msec);

 speedLB = 50;

  tPosLB = 360-200;

  wait(800,msec);

  speedLB = 1;

  tPosLB = 1;
}







void autonTwo(){

  
  
  logD("RUN AUTON2", "Running auton 2");
  
  setup();

  odom_constants();

  chassis.set_heading(180);
  chassis.desired_heading=180;

  vex::task flatEarth = vex::task(spinToPosTask);
  speedLB=0.7;
  tPosLB = 360-346;


  driveDistance(-22);//-35);
  driveDistance(-6);
  driveDistance(-4);

  clampPistons.set(true);
  wait(40,msec);
  
  turnToHeadingC(-270);
  speedLB=1;

  Intake.setVelocity(100, pct);
  
  driveDistance(13);
  wait(15,msec);
  driveDistance(4);

  wait(350,msec);


  turnToHeadingC(0);


  driveDistance(13);
  
  driveDistance(3);

  wait(140,msec);

  driveDistance(-12);

  //driveDistance(-18);
  //driveDistance(6);

  //turnToHeadingC(-13);
  turnToHeadingC(28);
  
  //driveDistance(9);
  driveDistance(9);
  driveDistance(4);
  wait(40,msec);
  driveDistance(-4);
  turnToHeadingC(214);

  

  driveDistance(50);

  Intake.setVelocity(-100, percent);

  turnToHeadingC(270);

  driveDistance(26);

  driveDistance(-5.5-3);
  
  
 turnToHeadingC(180);

 setMotorVelocities(20);

 wait(350,msec);

 speedLB = 50;

  tPosLB = 360-200;

  wait(800,msec);

  speedLB = 1;

  tPosLB = 1;
}

// void autonTwo(){

  
  
//   logD("RUN AUTON2", "Running auton 2");
  
//   setup();

//   odom_constants();

//   chassis.set_heading(180);
//   chassis.desired_heading=180;

//   vex::task flatEarth = vex::task(spinToPosTask);
//   speedLB=0.7;
//   tPosLB = 360-346;


//   driveDistance(-22);//-35);
//   driveDistance(-6);
//   driveDistance(-4);

//   clampPistons.set(true);
  
//   turnToHeadingC(-270);
//   speedLB=1;

//   Intake.setVelocity(100, pct);
  
//   driveDistance(13);
//   wait(15,msec);
//   driveDistance(4);

//   wait(350,msec);


//   turnToHeadingC(0);


//   driveDistance(13);
  
//   driveDistance(3);

//   wait(140,msec);

//   driveDistance(-12);

//   //turnToHeadingC(-13);
//   turnToHeadingC(28);
  

//   driveDistance(8);
//   driveDistance(4);
//   wait(40,msec);
//   driveDistance(-4);
//   turnToHeadingC(232);

  

//   driveDistance(24);
//   raiseIntake.set(true);
//   driveDistance(24);

//   raiseIntake.set(false);

//   wait(50,msec);

//   driveDistance(-3);

//   wait(100,msec);
  

//   turnToHeadingC(223);
  
//   Intake.setVelocity(-60, percent);

//   driveDistance(21);

  

//   driveDistance(-8);

// Intake.setVelocity(100, percent);

//   turnToHeadingC(180);

//   setMotorVelocities(20);

// wait(100,msec);

//   speedLB = 10;

//  tPosLB = 360-200;


//   wait(10,sec);

//   turnToHeadingC(150);

//   dPistons.set(true);
//   // chassis.left_swing_to_angle(180);
//   setMotorVelocities(0);
//   RightMotors.setVelocity(100, percent);

//   wait(60,msec);

//   dPistons.set(false);
//   setMotorVelocities(100);

//   wait(20,msec);

//   speedLB = 10;

//  tPosLB = 360-200;
  
 
// }

void autonThree(){
  //Right SIDE AWP for red
  logD("RUN AUTON3", "Running auton 3");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);  
  //tPosLB = 60; 
  int reverse = -1;//-1 is true, 1 is false
 
  vex::task flatEarth = vex::task(spinToPosTask);

  chassis.set_coordinates(0, 3, 0);

  //driveDistance(-28.5);

  //driveDistance(-10);

  peerPursuit(2000, 0, -1, 2);//2 is reversed

  setMotorVelocities(0);

  wait(10,msec);

  //turnToHeadingC(30);

  //driveDistance(-11);

  turnToHeadingC(-30);
  turnToHeadingC(-30);

  setMotorVelocities(-13);

  wait(300,msec);

  driveToLine(false, 2000);
  setMotorVelocities(0);
  clampPistons.set(true);

  wait(20,msec);
  Intake.setVelocity(100, percent);

  driveDistance(18);

  //wait(50,msec);

  turnToHeadingC(reverse*-20);

  Intake.setVelocity(0, percent);
  driveDistance(20);

  turnToHeadingC(reverse * -170);

  clampPistons.set(false);

  Intake.setVelocity(100, percent);
  driveDistance(12 + 4);
  //driveDistance(4);
  //driveDistance(4);
  driveDistance(4);
  driveDistance(4);

  wait(100,msec);

  //driveDistance(4);
  turnToHeadingC(reverse*90);
  wait(50,msec);
  turnToHeadingC(reverse*90);
  

  driveDistance(-15 - 11);
  //driveDistance(-4);
  //driveDistance(-4);
  //driveDistance(-4);
  driveDistance(-4);
  driveDistance(-4);

  clampPistons.set(true);

  wait(50,msec);
  //vex::task NYCSubwayPoisonGas = task(antiJamIntake);

  //wait(500,msec);
  raiseIntake.set(true);

  turnToHeadingC(reverse*-30);

  Intake.setVelocity(100, percent);

  driveDistance(20);

  

  driveDistance(4);
  driveDistance(4);
  wait(150,msec);
  raiseIntake.set(false);

  wait(500,msec);
  driveDistance(-12);
  wait(150,msec);
  driveDistance(16);

  wait(300,msec);

  driveDistance(-30);
  
  cancelIntake = true;

  tPosLB = 360-330;
  Intake.setVelocity(0, percent);
  //driveDistance(-20);

  turnToHeadingC(reverse*-160);
  Intake.setVelocity(100, percent);
  //driveDistance(16);
  
  
  setMotorVelocities(15);
  wait(3000,msec);
  setMotorVelocities(15);
}


void autonFour(){
  //Left SIDE AWP for blue
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  int reverse = 1;//-1 is true, 1 is false

 vex::task flatEarth = vex::task(spinToPosTask);

  chassis.set_coordinates(0, 3, 0);

  //driveDistance(-28.5);

  //driveDistance(-10);

  peerPursuit(2000, 0, -1, 1);//1 is other, 2 is reversed

  setMotorVelocities(0);

  wait(10,msec);

  //turnToHeadingC(30);

  //driveDistance(-11);

  setMotorVelocities(-30);

  wait(100,msec);

  driveToLine(false, 2000);
  setMotorVelocities(0);
  clampPistons.set(true);

  wait(20,msec);
  Intake.setVelocity(100, percent);

  driveDistance(18);

  //wait(50,msec);

  turnToHeadingC(reverse*-20);

  Intake.setVelocity(0, percent);
  driveDistance(20);

  turnToHeadingC(reverse * -170);

  clampPistons.set(false);

  Intake.setVelocity(100, percent);
  driveDistance(12 + 8);
  //driveDistance(4);
  //driveDistance(4);
  driveDistance(4);
  driveDistance(4);

  wait(100,msec);

  //driveDistance(4);
  turnToHeadingC(reverse*90);
  wait(50,msec);
  turnToHeadingC(reverse*90);
  

  driveDistance(-15 - 10);
  //driveDistance(-4);
  //driveDistance(-4);
  //driveDistance(-4);
  driveDistance(-4);
  driveDistance(-4);

  clampPistons.set(true);

  wait(50,msec);
  //vex::task NYCSubwayPoisonGas = task(antiJamIntake);

  //wait(500,msec);
  raiseIntake.set(true);

  turnToHeadingC(reverse*-45);

  Intake.setVelocity(100, percent);

  driveDistance(20);

  

  driveDistance(4);
  driveDistance(4);
  wait(150,msec);
  raiseIntake.set(false);

  wait(450,msec);
  driveDistance(-10);
  driveDistance(14);

  wait(300,msec);

  driveDistance(-26);

  
  turnToHeadingC(reverse*-160);
  
  //driveDistance(16);
  tPosLB = 360-330;
  
  setMotorVelocities(15);
  Intake.setVelocity(-100, percent);
  wait(3000,msec);
  setMotorVelocities(15);
}








void autonFourOld(){
  logD("RUN AUTON4", "Running auton 4");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;

  vex::task flatEarth = vex::task(spinToPosTask);
  //wait(5,sec);
  //driveDistance(-47);
  

  driveDistance(-21);
  wait(50,msec);

  turnToHeadingC(35);
  wait(50,msec);


  driveDistance(-12);

  clampPistons.set(true);

  wait(100,msec);

  Intake.setVelocity(100, percent);


  wait(200,msec);

  turnToHeadingC(90);

  clampPistons.set(false);
  driveDistance(24);

  driveDistance(4);

  wait(200,msec);

  Intake.setVelocity(0, percent);

  turnToHeadingC(45);

  driveDistance(-14);

  turnToHeadingC(326);

  driveDistance(-10.5);

  driveDistance(-4);

  clampPistons.set(true);

  wait(100,msec);

  Intake.setVelocity(100, percent);

  driveDistance(24);

  turnToHeadingC(225);
  tPosLB=100;

  setMotorVelocities(30);
  wait(2000,msec);
  setMotorVelocities(0);

}



void goalRushLeftV2(){
  logD("RUN GOALRUSH", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

 vex::task flatEarth = vex::task(spinToPosTask);

 driveDistance(40);


  turnToHeadingC(310);

  dPistons.set(true);

  wait(200,msec);

  turnToHeadingC(260);

  wait(100,msec);

  dPistons.set(false);

  wait(100,msec);

  turnToHeadingC(310);

  driveDistance(-10);

  driveDistance(-6);

  driveDistance(-6);

  clampPistons.set(true);

  wait(100,msec);

  Intake.setVelocity(100, percent);

  wait(1000,msec);

  clampPistons.set(false);

  wait(100,msec);

  turnToHeadingC(110);

  driveDistance(-22);

  driveDistance(-6);
  driveDistance(-6);

  clampPistons.set(true);

  wait(50,msec);

  turnToHeadingC(175);

  driveDistance(16);
  driveDistance(6);
  wait(200,msec);

  turnToHeadingC(270);
  driveDistance(-40);
  tPosLB=360-320;
}


void goalRushLeft(){
  logD("RUN GOALRUSH", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

 vex::task flatEarth = vex::task(spinToPosTask);

 driveDistance(40);


  turnToHeadingC(310);

  dPistons.set(true);

  wait(200,msec);

  turnToHeadingC(260);

  wait(100,msec);

  dPistons.set(false);

  wait(100,msec);

  turnToHeadingC(310);

  driveDistance(-10);

  driveDistance(-6);

  driveDistance(-6);

  clampPistons.set(true);

  wait(100,msec);

  Intake.setVelocity(100, percent);

  wait(1000,msec);

  clampPistons.set(false);

  wait(100,msec);

  turnToHeadingC(110);

  driveDistance(-22);

  driveDistance(-6);
  driveDistance(-6);

  clampPistons.set(true);

  wait(50,msec);

  driveDistance(18);

  turnToHeadingC(260);

  driveDistance(11);
  driveDistance(4);
  driveDistance(4);
  wait(200,msec);
  driveDistance(-6);
  driveDistance(6);
  wait(1000,msec);

  driveDistance(-40);

  tPosLB=360-320;
  
}


//DOnt use doesn't work sweeps cornor
void goalRushLeftSWEEP(){
  logD("RUN GOALRUSH", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

 vex::task flatEarth = vex::task(spinToPosTask);

 driveDistance(40);


  turnToHeadingC(310);

  dPistons.set(true);

  wait(200,msec);

  turnToHeadingC(260);

  wait(100,msec);

  dPistons.set(false);

  wait(100,msec);

  turnToHeadingC(310);

  driveDistance(-10);

  driveDistance(-6);

  driveDistance(-6);

  clampPistons.set(true);

  wait(100,msec);

  Intake.setVelocity(100, percent);

  wait(1000,msec);

  clampPistons.set(false);

  wait(100,msec);

  turnToHeadingC(110);

  driveDistance(-22);

  driveDistance(-6);
  driveDistance(-6);

  clampPistons.set(true);

  wait(50,msec);

  turnToHeadingC(175);

  driveDistance(16);
  driveDistance(6);
  wait(200,msec);

  // driveDistance(22);

  // wait(100,msec);

  turnToHeadingC(200);

  dPistons.set(true);

  driveDistance(25);

  turnToHeadingC(140);

  

  dPistons.set(false);

  wait(50,msec);

  turnToHeadingC(170);

  setMotorVelocities(20);
  wait(1.5,seconds);
  
}


void goalRushRight(){
  logD("RUN GOALRUSH", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

 vex::task flatEarth = vex::task(spinToPosTask);

 driveDistance(40);


  turnToHeadingC(-350);

  dPistons.set(true);

  wait(280,msec);

  driveDistance(-15);

  wait(100,msec);

  dPistons.set(false);

  wait(100,msec);

  turnToHeadingC(-260);

  driveDistance(-14);

  driveDistance(-4);

  driveDistance(-4);

  clampPistons.set(true);

  wait(100,msec);

  driveDistance(12);

  Intake.setVelocity(100, percent);

  wait(650,msec);

  clampPistons.set(false);

  wait(100,msec);

  turnToHeadingC(-142);

  driveDistance(-17);

  driveDistance(-4);
  driveDistance(-4);

  clampPistons.set(true);

  wait(50,msec);

  driveDistance(18);

  turnToHeadingC(-275);

  driveDistance(18);
  driveDistance(4);
  wait(100,msec);
  setMotorVelocities(-100);
  wait(150,msec);
  setMotorVelocities(100);
  wait(100,msec);
  setMotorVelocities(0);
  wait(650,msec);

  //turnToHeadingC(-250);

  //driveDistance(-37);

  //tPosLB=360-320;

  setMotorVelocities(-100);
  wait(250,msec);
  setMotorVelocities(0);
  wait(250,msec);
  
  setMotorVelocities(40);
  wait(300,msec);
  setMotorVelocities(0);
  wait(250,msec);
  
  
}






//NOT SAWP, it does aliance and scores 3 rings on a goal
void ringsRight(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 90);
  chassis.desired_heading = 90;

 vex::task flatEarth = vex::task(spinToPosTask);

 Intake.setVelocity(0, percent);

 speedLB = 0.5;

 tPosLB = 360-290;

 driveDistance(-17);

 turnToHeadingC(180);

 setMotorVelocities(20);

 wait(250,msec);

 speedLB = 10;

 tPosLB = 360-220;

 wait(320,msec);

 driveDistance(-2.5);

 speedLB = 1;

 tPosLB = 1;

 turnToHeadingC(216);
 //turnToHeadingC(216);

 driveDistance(-30);
 //wait(2.5,sec);
 driveDistance(-10);

 //setMotorVelocities(-35);
 //wait(50,msec);

 clampPistons.set(true);

 setMotorVelocities(0);



  //wait(20,sec);

  Intake.setVelocity(0, percent);

 turnToHeadingC(45);
 //turnToHeadingC(45);

 Intake.setVelocity(100, percent);

 driveDistance(16.75);

 //turnToHeadingC(140);
 chassis.right_swing_to_angle(115);

 driveDistance(18);

 wait(50,msec);

 turnToHeadingC(245);


  driveDistance(20);
  driveDistance(20);
  turnToHeadingC(270);
  driveDistance(17);


  clampPistons.set(false);

  turnToHeadingC(150);
  driveDistance(-10);
  driveDistance(-5);
  driveDistance(-5);

  clampPistons.set(true);

  turnToHeadingC(270);

  Intake.setVelocity(100, percent);

  driveDistance(30);

  
  wait(5,sec);
  //chassis.left_swing_to_angle(80);

 
 
}


void ringsLeft(){
  logD("RUN RSL", "Running rings safety left");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 270);
  chassis.desired_heading = 270;

 vex::task flatEarth = vex::task(spinToPosTask);

 Intake.setVelocity(0, percent);

 speedLB = 0.5;

 tPosLB = 360-290;

 driveDistance(-17);

 turnToHeadingC(-180);

 setMotorVelocities(20);

 wait(250,msec);

 speedLB = 10;

 tPosLB = 360-220;

 wait(320,msec);

 driveDistance(-2.5);

 speedLB = 1;

 tPosLB = 1;

 turnToHeadingC(-216);
 //turnToHeadingC(216);

 driveDistance(-30);
 //wait(2.5,sec);
 driveDistance(-10);

 //setMotorVelocities(-35);
 //wait(50,msec);

 clampPistons.set(true);

 setMotorVelocities(0);



  //wait(20,sec);

  Intake.setVelocity(0, percent);

 turnToHeadingC(-45);
 //turnToHeadingC(45);

 Intake.setVelocity(100, percent);

 driveDistance(16.75);

 //turnToHeadingC(140);
 chassis.left_swing_to_angle(-115);

 driveDistance(18);

 wait(50,msec);

 turnToHeadingC(-245);


  driveDistance(20);
  driveDistance(20);
  turnToHeadingC(-270);
  driveDistance(17);


  clampPistons.set(false);

  turnToHeadingC(-150);
  driveDistance(-10);
  driveDistance(-5);
  driveDistance(-5);

  clampPistons.set(true);

  turnToHeadingC(-270);

  Intake.setVelocity(100, percent);

  driveDistance(30);

  
  wait(5,sec);
  //chassis.left_swing_to_angle(80);

 
 
}




void LeftSafeFour(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  //int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 270);
  chassis.desired_heading = 270;

  

 vex::task flatEarth = vex::task(spinToPosTask);

 Intake.setVelocity(0, percent);

 speedLB = 0.5;

 tPosLB = 360-290;

 driveDistance(-17-1.5);

 turnToHeadingC(-180);

 setMotorVelocities(20);

 wait(250,msec);

 speedLB = 10;

 tPosLB = 360-200;

 wait(320,msec);

 driveDistance(-3.2);

 speedLB = 1;

 tPosLB = 1;

 turnToHeadingC(-216);
 ///turnToHeadingC(216);

 driveDistance(-30);
 //wait(2.5,sec);
 driveDistance(-10);

 //setMotorVelocities(-35);
 //wait(50,msec);

 clampPistons.set(true);

 setMotorVelocities(0);



  //wait(20,sec);

  Intake.setVelocity(0, percent);

 turnToHeadingC(-45);
 //turnToHeadingC(45);

 Intake.setVelocity(100, percent);

 driveDistance(19);

 wait(600,msec);

 //turnToHeadingC(140);
 chassis.left_swing_to_angle(-125);

 driveDistance(18);

 wait(50,msec);
 wait(700,msec);

 turnToHeadingC(-20);


 driveDistance(10);
 driveDistance(4);

 wait(700,msec);

 driveDistance(-10);

 turnToHeadingC(-270);
 driveDistance(40);
 tPosLB = 360-300;
 wait(5000,msec);
 setMotorVelocities(00);
}



//NOT SAWP, it does aliance and scores 3 rings on a goal
void RightSafeFour(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  //int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 90);
  chassis.desired_heading = 90;

  

 vex::task flatEarth = vex::task(spinToPosTask);

 Intake.setVelocity(0, percent);

 speedLB = 0.5;

 tPosLB = 360-290;

 driveDistance(-17-1.5);

 turnToHeadingC(180);

 setMotorVelocities(20);

 wait(250,msec);

 speedLB = 10;

 tPosLB = 360-200;

 wait(320,msec);

 driveDistance(-3.2);

 speedLB = 1;

 tPosLB = 1;

 turnToHeadingC(216);
 ///turnToHeadingC(216);

 driveDistance(-30);
 //wait(2.5,sec);
 driveDistance(-10);

 //setMotorVelocities(-35);
 //wait(50,msec);

 clampPistons.set(true);

 wait(50,msec);

 setMotorVelocities(0);



  //wait(20,sec);

  Intake.setVelocity(0, percent);

 turnToHeadingC(45);
 //turnToHeadingC(45);

 Intake.setVelocity(100, percent);

 driveDistance(19);

 wait(600,msec);

 //turnToHeadingC(140);
 chassis.right_swing_to_angle(125);

 driveDistance(18);

 wait(50,msec);
 wait(700,msec);

 turnToHeadingC(20);


 driveDistance(9);
 driveDistance(4);

 wait(500,msec);

 driveDistance(-10);

 turnToHeadingC(270);
 driveDistance(40);
 tPosLB = 360-300;
 wait(5000,msec);
 setMotorVelocities(00);
}









void leftQuickRush(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  //int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

  

 vex::task flatEarth = vex::task(spinToPosTask);

 MMM.set(true);

 setMotorVelocities(100);
 wait(845,msec);
 MMM.set(false);

  // setMotorVelocities(-20);
  // LeftMotors.setVelocity(-100, percent);
  // wait(800,msec);
  // setMotorVelocities(0);


 //wait(50,sec);
 
 
 
 setMotorVelocities(-100);
 wait(1100,msec);

// setMotorVelocities(0);
// chassis.turn_to_point(18, 30, 180);
// float dint = sqrt( pow(18-chassis.get_X_position(), 2) + pow(30-chassis.get_Y_position(), 2)  );
// driveDistance(-dint + 2);

turnToHeadingC(213);
driveDistance(-19);

driveDistance(-6);
driveDistance(-5);
clampPistons.set(true);

wait(80,msec);

Intake.setVelocity(100, percent);

driveDistance(29);
wait(600,msec);
// turnToHeadingC(286);
// driveDistance(24);
// turnToHeadingC(20);

turnToHeadingC(346);

driveDistance(26.5);
driveDistance(6);
driveDistance(6);
wait(100,msec);
driveDistance(-10);
wait(100,msec);
driveDistance(6);

wait(300,msec);
driveDistance(-14);
wait(100,msec);
driveDistance(10);
}



void rightQuickRush(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  //ladyBrownTrack.setPosition(153, degrees);
  //tPosLB = 60;
  //int reverse = 1;//-1 is true, 1 is false

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;

  

 vex::task flatEarth = vex::task(spinToPosTask);

 MMM.set(true);

 setMotorVelocities(100);
 wait(845,msec);
 MMM.set(false);

  // setMotorVelocities(-20);
  // LeftMotors.setVelocity(-100, percent);
  // wait(800,msec);
  // setMotorVelocities(0);


 //wait(50,sec);
 
 
 
 setMotorVelocities(-100);
 wait(1100,msec);

// setMotorVelocities(0);
// chassis.turn_to_point(18, 30, 180);
// float dint = sqrt( pow(18-chassis.get_X_position(), 2) + pow(30-chassis.get_Y_position(), 2)  );
// driveDistance(-dint + 2);

turnToHeadingC(-213);
driveDistance(-19);

driveDistance(-6);
driveDistance(-5);
clampPistons.set(true);

wait(80,msec);

Intake.setVelocity(100, percent);

driveDistance(29);
wait(600,msec);
// turnToHeadingC(286);
// driveDistance(24);
// turnToHeadingC(20);

turnToHeadingC(-346);

driveDistance(26.5);
driveDistance(6);
driveDistance(6);
wait(100,msec);
driveDistance(-10);
wait(100,msec);
driveDistance(6);

wait(300,msec);
driveDistance(-14);
wait(100,msec);
driveDistance(10);
}











void newRightRush(){

  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, -125);
  chassis.desired_heading = -125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-26-1.5);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(-210);

  

  donaldDuck(-7);

  wait(50,msec);

  call4DD(-12, 10.5, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(-262);

  setTargetIntake(100);

  //colorSort = true;

  driveDistance(27);

  wait(2000,msec);


   

  //dPistons.set(true);

  turnToHeadingC(-262+180);

  tPosLB = 140;

  // donaldDuck(36);

  // tPosLB = 40;

  //donaldDuck(24);

  

  setMotorVelocities(100);
  wait(500,msec);
  LeftMotors.stop(coast);
  RightMotors.stop(coast);
  

  


}
void newRightRushNOTSAFE(){

  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, -125);
  chassis.desired_heading = -125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-26-1.5);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(-210);

  

  donaldDuck(-7);

  wait(50,msec);

  call4DD(-12, 10.5, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(-310);

  //dPistons.set(true);

   call4DD(14.5, 8.5, intakeForwards);
  //call4DD(6.5, 3.5, intakeForwards);

  setTargetIntake(100);

  wait(200,msec);

  turnToHeadingC(-274);

  donaldDuck(20);

  wait(200,msec);

  donaldDuck(-25);

  wait(200,msec);

  turnToHeadingC(-234);

  donaldDuck(22);

  turnToHeadingC(-70);

  // donaldDuck(36);

  // tPosLB = 40;

  donaldDuck(24);

  tPosLB = 140;

  // setMotorVelocities(100);
  // wait(400,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);
  

  


}


void newLeftRush(){

  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 125);
  chassis.desired_heading = 125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);
  colorSort=true;

  //driveDistance(24);
  //donaldDuck(24);

  setTargetIntake(100);
  turnToHeadingC(125+90);

  wait(20,sec);
  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-25.5);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(210);

  

  donaldDuck(-9);

  wait(50,msec);

  call4DD(-10, 6.5, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(262);

  setTargetIntake(100);

  colorSort = true;

  driveDistance(27);

  wait(2000,msec);

  colorSort = false;


  //tPosLB = 140;

  //dPistons.set(true);

  //turnToHeadingC(360-(-262+180-30));

  turnToHeadingC(360-(-262+180-25-4));

  //tPosLB = 140;

  // donaldDuck(36);

    setTargetIntake(0);

  // tPosLB = 40;

  donaldDuck(34);

  

  // setMotorVelocities(100);
  // wait(500,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);
  

  // setMotorVelocities(100);
  // wait(500,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);

}
void newLeftRushNOTSAFE(){

  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 125);
  chassis.desired_heading = 125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);
    // turnToHeadingC(125+90);
    // wait(25,sec);

  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-24);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(210);

  

  donaldDuck(-7);

  wait(50,msec);

  call4DD(-12, 9, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(303);


  //dPistons.set(true);

  setTargetIntake(100);

   call4DD(16.0, 4.5, intakeForwards);
  //call4DD(6.5, 3.5, intakeForwards);

  setTargetIntake(100);

  wait(200*000,msec);

  turnToHeadingC(277);

  colorSort = true;

  donaldDuck(14);

  wait(1000,msec);

  colorSort = false;

  donaldDuck(-14);

  wait(200,msec);

  turnToHeadingC(220);

  donaldDuck(22);

  wait(1000,msec);

  turnToHeadingC(70);

  // donaldDuck(36);

  // tPosLB = 40;

  donaldDuck(23);

  tPosLB = 140;

  // setMotorVelocities(100);
  // wait(400,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);
  

  

}




void newRightGoal(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, -125);
  chassis.desired_heading = -125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);
    // turnToHeadingC(125+90);
    // wait(25,sec);

  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-24);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(-210);

  

  donaldDuck(-7);

  wait(50,msec);

  call4DD(-12, 9, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(-303);


  //dPistons.set(true);

  setTargetIntake(100);

   call4DD(16.0, 4.5, intakeForwards);
  //call4DD(6.5, 3.5, intakeForwards);

  setTargetIntake(100);

  wait(200*000,msec);

  turnToHeadingC(-277);

  colorSort = true;

  donaldDuck(14);

  wait(1000,msec);

  colorSort = false;

  donaldDuck(-14);

  wait(200,msec);

  turnToHeadingC(-220);

  donaldDuck(22);

  wait(1000,msec);

  turnToHeadingC(-70);

  // donaldDuck(36);

  // tPosLB = 40;

  donaldDuck(23);

  tPosLB = 140;

  // setMotorVelocities(100);
  // wait(400,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);
  

  
}


void newLeftGoal(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;


  vex::task earthIntake = vex::task(runIntake);

  donaldDuck(-9);


  turnToHeadingC(-330);

  donaldDuck(-11);

  call4DD(-12, 10, clampTrue);

  clampPistons.set(true);

  wait(0.2,sec);
  setTargetIntake(100);
  wait(1,sec);


  turnToHeadingC(-280);

  setTargetIntake(100);

  donaldDuck(26);

  wait(150,msec);

  donaldDuck(-10);

}




void rushNEwNew(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 0);
  chassis.desired_heading = 0;


  vex::task earthIntake = vex::task(runIntake);

  donaldDuck(-9);


  turnToHeadingC(330);

  donaldDuck(-11);

  call4DD(-12, 10, clampTrue);

  clampPistons.set(true);

  wait(0.2,sec);
  setTargetIntake(100);
  wait(1,sec);


  turnToHeadingC(280);

  setTargetIntake(100);

  donaldDuck(26);

  wait(150,msec);

  donaldDuck(-10);

}








void leftRingFour(){

  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 125);
  chassis.desired_heading = 125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);

  
  tPosLB = 165;

  wait(600,msec);
  // donaldDuck(-14);

  // tPosLB = 1;
  // wait(150,msec);

  // turnToHeadingC(180);

  

  // donaldDuck(-11);

  // call4DD(-12, 9, clampTrue);


  donaldDuck(-27);

  tPosLB = 1;
  wait(150,msec);

  turnToHeadingC(210, "BAD");

  

  chassis.drive_to_point(-6, 28);

  wait(50,msec);

  call4DD(-12, 9, clampTrue);

  clampPistons.set(true);


  turnToHeadingC(310);


  //dPistons.set(true);

   call4DD(14.5, 8.5, intakeForwards);
  //call4DD(6.5, 3.5, intakeForwards);

  setTargetIntake(100);

  wait(200,msec);

  turnToHeadingC(277);

  donaldDuck(20);

  wait(500,msec);

  donaldDuck(-20);

  wait(200,msec);

  turnToHeadingC(220);

  donaldDuck(22);

  wait(1000,msec);

  turnToHeadingC(70);

  // donaldDuck(36);

  // tPosLB = 40;

  donaldDuck(22);

  tPosLB = 140;

  // setMotorVelocities(100);
  // wait(400,msec);
  // LeftMotors.stop(coast);
  // RightMotors.stop(coast);
  

  
}


void leftGoalRUSH(){
  logD("RUN AUTON4", "Running auton 4 new");

  setup();

  odom_constants();

  chassis.set_coordinates(0, 0, 125);
  chassis.desired_heading = 125;

  vex::task flatEarth = vex::task(spinToPosTask);
  vex::task earthIntake = vex::task(runIntake);
  tPosLB = 120;
  driveDistance(34);
  
  tPosLB = 200;

  wait(200,msec);
  driveDistance(-24);
}












