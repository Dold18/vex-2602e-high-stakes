#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "info/log.h"
#include "DriveFuncs/PID.h"
 
#include <string>
#include <array>
#include <random>

int plainTurnProfile(float difference){
  int ext = 1500;
  if(difference<20){
    //12
    ext = 680;
    chassis.set_turn_constants(12, .45, 0.01648, 3.31782, 11.7779);
  }
  else if(difference<40){
    //30
    ext = 700;
    chassis.set_turn_constants(12, .37841, 0.02269, 3.246, 13.4047);
  }
  else if(difference<60){
    //45 
    ext = 800;
    chassis.set_turn_constants(12, 0.379556,  0.0468, 2.94, 12.489);
    //bad chassis.set_turn_constants(12, .4516, .0046, 3.0105, 15.014);
  }
  else if (difference<110) {
    //90
    ext = 1000;
    chassis.set_turn_constants(12, 0.454796, .0618, 3.3077, 13.64);
  }
  else if (difference<155) {//SHOULD BE 155
    //135
    ext = 1200;
    chassis.set_turn_constants(12, 0.3144, 0.05137, 2.56387, 13.319);
  }
  else if (difference<200) {
    //180
    ext = 1300;
    chassis.set_turn_constants(12, 0.3617, 0.0387, 3.327, 12.27);
  }
  return ext;

}

int mobileGoal0Profile(float difference){
  int ext = 1500;
  if(difference<20){
    //12
    ext = 600;
    chassis.set_turn_constants(12, .4166, 0.0062, 3.306, 16.4);
  }
  else if(difference<40){
    //30
    ext = 700;
    chassis.set_turn_constants(12, .3457, 0.0185, 2.697, 13.205);
  }
  else if(difference<60){
    //45 
    ext = 800;
    chassis.set_turn_constants(12, 0.36,  0.01567, 3.259, 18.58);
    //bad chassis.set_turn_constants(12, .4516, .0046, 3.0105, 15.014);
  }
  else if (difference<110) {
    //90
    ext = 1000;
    chassis.set_turn_constants(12, 0.4061, .01898, 2.742, 12.52);
  }
  else if (difference<155) {//SHOULD BE 155
    //135
    ext = 1100;
    chassis.set_turn_constants(12, 0.42, 0.041, 2.8, 13.3);
  }
  else if (difference<200) {
    //180
    ext = 1500;
    chassis.set_turn_constants(12, 0.34, 0.0348, 3.187, 17.91);
  }

  return ext;
}

int mobileGoal3Profile(float difference){
  int ext = 1500;
  if(difference<20){
    //12  
    ext = 700;
    chassis.set_turn_constants(12, .354139, 0.0093, 2.6, 14.15);
  }
  else if(difference<40){
    //30
    ext = 800;
    chassis.set_turn_constants(12, .424, 0.02882, 3.156, 13.72);
  }
  else if(difference<60){
    //45 
    ext = 700;
    chassis.set_turn_constants(12, 0.4419,  0.0132171, 3.09055, 11.54);
    //bad chassis.set_turn_constants(12, .4516, .0046, 3.0105, 15.014);
  }
  else if (difference<110) {
    //90
    ext = 930;
    chassis.set_turn_constants(12, 0.44528, 0.0434739, 3.32, 11.4431);
  }
  else if (difference<155) {//SHOULD BE 155
    //135
    ext = 1000;
    chassis.set_turn_constants(12, 0.300609, 0.00934, 2.589, 16.63);
  }
  else if (difference<200) {
    //180
    ext = 1200;
    chassis.set_turn_constants(12, 0.345, 0.0430636, 3.5328, 14.4568);
  }

  return ext;
}

int mobileGoal0_LB90_Profile(float difference){
  int ext = 1500;
  if(difference<20){
    //12
    ext = 650;
    chassis.set_turn_constants(12, .4362, 0.019322, 3.36, 16.25);
  }
  else if(difference<40){
    //30
    ext = 600;
    chassis.set_turn_constants(12, .5592, 0.01426, 3.35764, 14.6794);
  }
  else if(difference<60){
    //45 
    ext = 650;
    chassis.set_turn_constants(12, 0.4638,  0.004267, 3.47469, 17.841);
    //bad chassis.set_turn_constants(12, .4516, .0046, 3.0105, 15.014);
  }
  else if (difference<110) {
    //90
    ext = 900;
    chassis.set_turn_constants(12, 0.516274, .00435358, 3.2949, 14.1953);
  }
  else if (difference<155) {//SHOULD BE 155
    //135
    ext = 1080;
    chassis.set_turn_constants(12, 0.36835, 0.0457, 2.85574, 17.285);
  }
  else if (difference<200) {
    //180
    ext = 1300;
    chassis.set_turn_constants(12, 0.3771, 0.05, 3.4853, 12.379);
  }

  return ext;
}



int mobileGoal2_LB90_Profile(float difference){
  int ext = 1500;
  if(difference<20){
    //12
    ext = 650;
    chassis.set_turn_constants(12, .39568, 0.005, 3.1755, 18);
  }
  else if(difference<40){
    //30
    ext = 650;
    chassis.set_turn_constants(12, .5492, 0.02315, 3.46, 11.63);
  }
  else if(difference<60){
    //45 
    ext = 700;
    chassis.set_turn_constants(12, 0.4012,  0.00543, 3.0643, 18.56);
    //bad chassis.set_turn_constants(12, .4516, .0046, 3.0105, 15.014);
  }
  else if (difference<110) {
    //90 //may want to redo
    ext = 920;
    chassis.set_turn_constants(12, 0.4796, .072135, 3.4179, 11.495);
  }
  else if (difference<155) {//SHOULD BE 155
    //135
    ext = 1080;
    chassis.set_turn_constants(12, 0.297338, .01764, 2.535, 13.762);
  }
  else if (difference<200) {
    //180
    ext = 1250;
    chassis.set_turn_constants(12, 0.32, 0.0166572, 2.5186, 15.15);
  }

  return ext;
}



//Takes in the angle we should currently be at
//Has different pid values for every angle 
float turnToHeadingC(float tHeading, std::string acc){


  //isTurning=true;  

  float currentHeading = chassis.get_absolute_heading();


  float difference = tHeading - currentHeading;
  //try to find shortest turn
  float differenceV2 = (tHeading+360) - currentHeading;

  if(difference<0){
    difference = -difference;
  }
  if(differenceV2<0){
    differenceV2 = -differenceV2;
  }
  if(difference > differenceV2){
    difference = differenceV2; //this turn is faster
  }

  logD("START CUSTOM TURN", "turnHeading:" + FTS(tHeading) + ", currentHeading:" + FTS(currentHeading) + ", difference" + FTS(difference));

  //chassis.set_turn_exit_conditions(1, 180, 1000);
  //bAD


  int ext = 0;
  if(acc=="MobileGoal0"){
    ext = mobileGoal0Profile(difference);
  }
  if(acc=="MobileGoal3"){
    ext = mobileGoal3Profile(difference);
  }
  else if(acc=="MobileGoal0_LB90"){
    ext = mobileGoal0_LB90_Profile(difference);
  }
  else if(acc=="MobileGoal2_LB90"){
    ext = mobileGoal2_LB90_Profile(difference);
  }
  else{//PLAIN
    ext = plainTurnProfile(difference);
  }



  float startingTime = Brain.timer(msec);

  if(acc=="ULTRA"){
    chassis.set_turn_exit_conditions(0.8, 500, ext+200);
  }
  else if(acc=="LOW"){
    chassis.set_turn_exit_conditions(2, 150, ext-200);
  }
  else if(acc=="MED"){
    chassis.set_turn_exit_conditions(1.4, 250, ext-120);
  }
  else if(acc=="DOG"){
    chassis.set_turn_exit_conditions(3, 120, ext-250);
  }
  else{//HIGH
    chassis.set_turn_exit_conditions(1, 300, ext);
  }

  chassis.turn_to_angleAntiO(tHeading);
  //THC(tHeading);
  logD("LOG ANGLE", FTS(difference));
  
  logD("STOP CUSTOM TURN", "New Current Heading:" + FTS(currentHeading + tHeading) + "Elapsed Time:" + FTS(Brain.timer(msec) - startingTime) + "Accuracy:" + FTS(chassis.get_absolute_heading()-tHeading));

  //isTurning=false; 

  return currentHeading + tHeading;
}



float turnToHeadingC(float tHeading){
  return turnToHeadingC(tHeading, "HIGH");
}


void setDriveParams(float inches){
  float absInches;
  if(inches<0){
    absInches = -inches;
  }
  else{
    absInches = inches;
  }

  logD("START CUSTOM TURN", "inches:" + FTS(inches) + ", absInches:" + FTS(absInches));
  
  if(absInches<7){
    //6in
    chassis.set_drive_constants(10, 1.748, .00, 10.0517, 0);
  }
  else if(absInches<11){
    //10 in 680
    chassis.set_drive_constants(10, 1.6294, .02678, 10.61, 5.46);
  }
  else if(absInches<20){
    //15
    chassis.set_drive_constants(10, 1.405, .06552, 9.515, 0);
  }
  else if(absInches<32){
    //24 in 910
    chassis.set_drive_constants(10, 1.641, 0.04894, 9.7373, 1.5);
  }
  else if(absInches<48){
    //40 in 1100
    chassis.set_drive_constants(10, 1.7812, .0402, 9.3187, 9.165);
  }
  else{
    //60+ NOT TUNED
    chassis.set_drive_constants(10, 1.68315, -0.033757, 9.9326, -0.114332);
  }
  // else if(absInches<55){
  //   //50 in
  //   //chassis.set_drive_constants(10, , , , );
  // }
  // else if(absInches<65){
  //   //60 in
  //   //chassis.set_drive_constants(10, , , , );
  // }
  // else{//All others 35+
  //   //40+ in

  // }

  chassis.set_drive_exit_conditions(1, 300, 3000);

  //float startingTime = Brain.timer(msec);

  //chassis.drive_distance(inches);
  //donaldDuck(inches);

  //logD("STOP CUSTOM DRIVE", "Elapsed Time:" + FTS(Brain.timer(msec) - startingTime));
  // return inches;
}

void driveDistance(float inches){
  setDriveParams(inches);

  chassis.set_drive_exit_conditions(1, 300, 3000);

  //float startingTime = Brain.timer(msec);

  //chassis.drive_distance(inches);
  donaldDuck(inches);

  //logD("STOP CUSTOM DRIVE", "Elapsed Time:" + FTS(Brain.timer(msec) - startingTime));
  // return inches;
}



float getAnglePoint(float X_position, float Y_position){
  return to_deg( atan2(X_position-chassis.get_X_position(),Y_position-chassis.get_Y_position()) );
}