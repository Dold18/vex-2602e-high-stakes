// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LF                   motor         9               
// LM                   motor         6               
// LB                   motor         7               
// RF                   motor         17              
// RM                   motor         16              
// RB                   motor         15              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "botSetup/vex.h" 

#include <cmath> 

#include <string>

#include "info/tuneAble.h"

#include "info/settings.h"

#include "Libraries/brainUI.h"

#include "Libraries/controllerUI.h"

#include "Libraries/stringFunctions.h"

#include "Libraries/botErrorProcesses.h"
 
#include "DriveFuncs/motorFuncs.h"
 
#include "DriveFuncs/PSO_Turn.h"

#include "DriveFuncs/PSO_Drive.h"

#include "nonDriveFuncs/LadyBrown.h"

#include "botLab.h"

#include "info/log.h"

#include "autos/autons.h"

#include "autos/pointAutos.h"

#include "autos/tests.h"

#include "Sensing/colorSort.h"

#include "info/colors.h"

#include <functional>

#include <array>

using namespace vex;
competition Competition;






//I THINK YOUR LEAKING MEMORY LOL PROLLY SHOULD FIX THAT BEFORE YOU LOSE CUZ OF IT
//FIX MOTION WACK TRASH
///PURE DEF MEMORY LEAK FIX
//FIX LB MECH SHIT CODE BREAKS 
//LOG CODE BREAKS EVERYTHING FIX
//COLOR SORT DOESNT WORK FIX
//SWITCH TO LEMLIB
//DELETE JAR ITS TRASH



Drive chassis(

  //Specify your drive setup below. There are eight options:
  //ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
  //For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
  
  //ZERO_TRACKER_ODOM,
  TANK_TWO_ROTATION,//WRONG SETUP FOR TRACKING

  //Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
  //You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

  //Left Motors:
  motor_group(LeftBackMotor, LeftMiddleMotor, LeftFrontMotor),
  //LeftFrontMotor,

  //Right Motors:
  motor_group(RightBackMotor, RightMiddleMotor, RightFrontMotor),
  //RightFrontMotor,

  //Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
  PORT2,

  //Input your wheel diameter. (4" omnis are actually closer to 4.125"):
  2.802,//Little bit over 3.25

  //External ratio, must be in decimal, in the format of input teeth/output teeth.
  //If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
  //If the motor drives the wheel directly, this value is 1:
  1,

  //Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
  //For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
  360,

  /*---------------------------------------------------------------------------*/
  /*                                  PAUSE!                                   */
  /*                                                                           */
  /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
  /*  If you are not using position tracking, leave the rest of the values as  */
  /*  they are.                                                                */
  /*---------------------------------------------------------------------------*/

  //If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

  //FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
  //LF:      //RF:    
  PORT1, -PORT2,

  //LB:      //RB: 
  PORT3, -PORT4,

  //If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
  //If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
  //If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
  PORT5,

  //Input the Forward Tracker diameter (reverse it to make the direction switch):
  2.075,//IMPORTANT

  //Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
  //For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
  //This distance is in inches:
  0,//6.5,//IMPORTANT

  //Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
  PORT5,

  //Sideways tracker diameter (reverse to make the direction switch):
  2.075,//A little bigger than 2 inches based on testing

  //Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
  1.25

);


int current_auton_selection = 0;
bool auto_started = false;



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

std::string lastText = "";

int displayTempInfo(){
  std::string firstLine;
    std::string secondLine;
    std::string thirdLine;

    if (diagnostic == false) {
      std::string customValue = driveType;
      //std::string customValue = floatToString(potentiometer.value(pct));
      std::string tempMes = getTempMessage();
      std::string disMes = getDisconnectErrors();
      if (disMes == std::string("")) {
        disMes = getHeatMap();
      }

      firstLine = customValue;
      secondLine = tempMes;
      thirdLine = disMes;
    }
    if (diagnostic == true) {
      firstLine = getDiagnostic1();
      secondLine = getDiagnostic2();
      thirdLine = getDisconnectErrors();
      if (thirdLine == "") {
        thirdLine = getDiagnostic3();
      }
    }

    std::string newText = firstLine + secondLine + thirdLine;
    if (lastText != newText) {
      displayInfo(Controller1, firstLine, secondLine, thirdLine);
      wait(150,msec);
    }
    lastText = newText;

    return 0;
}




void userControl() {
  logD("START USERCONTROL", "", true);

  setup();

  //float currentTurn = 0;

  float axis1;
  float axis2;
  float axis3;
  float axis4;

  float RightMotorsVelocity;
  float LeftMotorsVelocity;

  float bMechPos = 0;
  bool clampThing = clampPistons.value();
  bool reL1;//Relase L1

  bool dingThing = false;
  bool reL2;//Relase L1

  bool raisedIntake = false;
  bool reX;

  bool MMMPISTON = false;
  bool reA;

  int counter = 0;


  int sorterCount = -1;
  

  bool LBDown = true;
  
  spinTask = false;

  

  lcdButton clampID = lcdButton(240, 100, 500, 300, "", brainColor, false);
  
  // if(teamColor=="NULL"){
  //   teamColor = readFile("/VEX/teamColor.txt");
  // }

  colorSort=false;
  if(tPosLB < 2){
    needReset = true;
  }

  while (true) {
    cancelIntake = true;
    speedLB = 1;
    runningProgram=true;
    antiGen = true;
    runLower=false;
    runTop=false;
    reverseLower = false;
    runLowerForward = false;
    
    //float rateOfTurn = Inertial.rotation(degrees);
    //currentTurn = Inertial.rotation(degrees);

    if (useCurve == true) {
      axis1 = runCurve(Controller1.Axis1.position(percent));
      axis2 = runCurve(Controller1.Axis2.position(percent));
      axis3 = runCurve(Controller1.Axis3.position(percent));
      axis4 = runCurve(Controller1.Axis4.position(percent));
    } else {
      axis1 = Controller1.Axis1.position(percent);
      axis2 = Controller1.Axis2.position(percent);
      axis3 = Controller1.Axis3.position(percent);
      axis4 = Controller1.Axis4.position(percent);
    }

    // if (intakeOverHeat() == false) {
    //   axis1 *= velocityCap;
    //   axis2 *= velocityCap;
    //   axis3 *= velocityCap;
    //   axis4 *= velocityCap;
    // }

    //driveType = "TANK";
    if (driveType == "THREEAXIS") {
      RightMotorsVelocity = (axis3 + (-axis1 * axis1Sense) + (-axis4 * axis4Sense));
      LeftMotorsVelocity = (axis3 - (-axis1 * axis1Sense) - (-axis4 * axis4Sense));
    } else if (driveType == "GTA") {
      //GTA drive
      RightMotorsVelocity = (axis3 + (-axis4 * axis4Sense));
      LeftMotorsVelocity = (axis3 - (-axis4 * axis4Sense));

    } else {
      //TANK drive if default on error
      LeftMotorsVelocity = axis3;
      RightMotorsVelocity = axis2;
    }
    //Tank
    //Good for notebook, basically mirrors both sides of the drivetrain,
    //so if one motor is disconnected then it is mirrored, some with overheating
    if ((driveType == "TANK") && (useJarDrive==true)) {
      float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
      float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
      LeftMotors.spin(fwd, to_volt(leftthrottle), volt);
      RightMotors.spin(fwd, to_volt(rightthrottle), volt);
    } 
    else if (motorErrorAutoCorrection == true) {
      LeftMotors.spin(forward);
      RightMotors.spin(forward);

      fixDriveMotors(LeftMotorsVelocity, RightMotorsVelocity);
    } 
    else {
      float leftthrottle = deadband(LeftMotorsVelocity, 5);
      float rightthrottle = deadband(RightMotorsVelocity, 5);
      LeftMotors.spin(fwd, to_volt(leftthrottle), volt);
      RightMotors.spin(fwd, to_volt(rightthrottle), volt);
    }

    if(counter>10){
      vex::task god = task(displayTempInfo);
      counter=0;
    }
    counter++;


    
      



    if (Controller1.ButtonR1.pressing()) {
      setTargetIntake(100);
      //extend = true;
    }
    else if (Controller1.ButtonR2.pressing()) {
      setTargetIntake(-100);
    }
    else{
      setTargetIntake(-0);
    }

    // if(colorSort == true){
    //   logD("LOG COLORSORT", "teamColor:" + teamColor + ", ringColor:" + getRingColor() + ", sorterCount" + FTS(sorterCount));
    //   if(getRingColor()!=teamColor && getRingColor()!="NONE"){
    //     sorterCount++;
    //     if(sorterCount >=1){
    //       EJECT = true;
    //       ejectCounter = 0;
    //     }
    //   }
    //   else{
    //     sorterCount = 0;
    //   }
    // }

    
    int lbTune = -1;
    if(Controller1.ButtonDown.pressing()){
      //tPosLB = 360-200 + lbTune; SCORE
      
      tPosLB = 360-214 + lbTune;
      setTargetIntake(-100);
      //LBDown = false;
    }
    if(Controller1.ButtonRight.pressing()){
      //spinToPosAsync(360-221.48);
      //tPosLB = 360-240 + lbTune; INTERMEDIATE
      // if(tPosLB<360-214){
      //   tPosLB = 360-214;
      // }
      tPosLB += 7;
      if(tPosLB>285){
        tPosLB = 285;
      }
      //intakeVelocity = -100;
      //Intake.setVelocity(-100, pct);
      
      //LBDown = false;
    }

    if(Controller1.ButtonY.pressing()){
      //spinToPosAsync(360-330.6);
      tPosLB = 360-340;
      //LBDown = true;
    }
    if(Controller1.ButtonB.pressing()){
      //spinToPosAsync(1);
      tPosLB = 1;
      needReset = true;
      //LBDown = true;
    }

    // if(LBDown==true){
    //   if(clampThing==true) tPosLB = 1;
    //   if(clampThing==false) tPosLB = 360-330.6;
      
    // }

    //spinToPos(bMechPos);

    if(Controller1.ButtonL1.pressing() && reL1){
      clampThing = !clampThing;
      if(clampThing==true){
        Controller1.rumble("-");
      }
      else{
        Controller1.rumble(".");
      }
      reL1 = false;//Relase
    }
    if( !Controller1.ButtonL1.pressing() ){
      reL1 = true;
    }

    if(Controller1.ButtonL2.pressing() && reL2){
      dingThing = !dingThing;
      reL2 = false;//Relase
    }
    if( !Controller1.ButtonL2.pressing() ){
      reL2 = true;
    }

    if(Controller1.ButtonX.pressing() && reX){
      //raisedIntake = !raisedIntake;
      //teamColor = "BLUE";
      
      colorSort= !colorSort;
      displayInfo(Controller1, BTS(colorSort));
      reX = false;//Relase
    }
    if( !Controller1.ButtonX.pressing() ){
      reX = true;
    }

    if(Controller1.ButtonA.pressing() && reA){
      MMMPISTON = !MMMPISTON;
      reA = false;//Relase
    }
    if( !Controller1.ButtonA.pressing() ){
      reA = true;
    }

    clampPistons.set(clampThing);
    dPistons.set(dingThing);
    if(Controller1.ButtonLeft.pressing()){
      raiseIntake.set(true);
    }
    else{
      raiseIntake.set(false);
    }
    //raisedIntake);
    
    if(clampThing==true){
      MMMPISTON=false;
    }
    MMM.set(MMMPISTON);

    // if(clampThing==true){
    //   brainColor = greenHex;
    // }
    // else{
    //   brainColor = pinkHex;
    // }

    clampID.setColor(brainColor);
    



    if (diagnostic == true) {
      driveType = "THREEAXIS";
      // if (Controller1.ButtonUp.pressing()) {
      //   driveType = "THREEAXIS";
      // } else if (Controller1.ButtonDown.pressing()) {
      //   driveType = "TANK";
      // } else if (Controller1.ButtonRight.pressing()) {
      //   driveType = "GTA";
      // }
    } else if (diagnostic == false) {
      driveType = defaultDriveType;
    }

    if (Controller1.ButtonUp.pressing()) {
      diagnostic = true;
    }
    if (Controller1.ButtonLeft.pressing()) {
      diagnostic = false;
    }



    //troll akshay

    if (trollAkshay == true) {

      // Seed the random number generator
      float randomNum = static_cast < float > (rand()) / (static_cast < float > (32767 / (1 - 0)));

      // Check if the random number is less than 0.02 (2%)
      if (randomNum < 300) {//Should be 30
        counter++;
        if (counter % 5 == 0) {
          Controller1.rumble("....");
        }
        if (counter % 5 == 1) {
          Controller1.rumble("- -.");
        }
        if (counter % 5 == 2) {
          Controller1.rumble("--");
        }
        if (counter % 5 == 3) {
          Controller1.rumble("-   .-");
        }
        if (counter % 5 == 4) {
          Controller1.rumble("-..     .-");
        }

      }
    }


    

    std::string axisString = "Axis 1: " + FTS(Controller1.Axis1.value()) + ", Axis 2: " + FTS(Controller1.Axis2.value()) + ", Axis 3: " + FTS(Controller1.Axis3.value()) + ", Axis 4: " + FTS(Controller1.Axis4.value());
    //std::string buttonString = "A" + Controller1.ButtonA
    std::string odomString = ", X: " + FTS(chassis.get_X_position()) + ", Y: " + FTS(chassis.get_Y_position()) + ", absHeading: " + FTS(chassis.get_absolute_heading());
    std::string motorString = ", intakeVelo:" + FTS(Intake.voltage());
    logD("STEP DRIVE", axisString + odomString + motorString);

    wait(25, msec);//Replaced with log

    if(spinTask==false) vex::task flatEarth = vex::task(spinToPosTask);
    if(intakeTask==false) vex::task earthIntake = vex::task(runIntake);
  }
  logD("STOP USERCONTROL", "", true);
}



//int auton = -1;

struct selectorButton { 
    std::string team;
    std::string name;
    std::function<void()> callback;
    lcdButton linkedButton = lcdButton(240, 100, 500, 300, "NuLL", greenHex, false);
};  

std::array<selectorButton, 10> autonButtons;

int autoCounter = 0;
void addAuton(int xi, int yi, int width, int height, std::string name, std::string team, std::function<void()> callback){
  autonButtons[autoCounter].name = name; 
  autonButtons[autoCounter].team = team;
  autonButtons[autoCounter].callback = callback;
  autonButtons[autoCounter].linkedButton.moveTo(xi, yi);
  autonButtons[autoCounter].linkedButton.setSize(width, height);
  autonButtons[autoCounter].linkedButton.setText(name);
  autoCounter++;
}

std::string auton = "NULL";

int selector(){

  addAuton(80, 155, 100, 100, "L Elims", "BLUE", pointGoalSideLeft);
  addAuton(170, 205, 100, 50, "L ElimsWS", "BLUE", pointLeftGoalElimWall);
  addAuton(170, 180, 100, 50, "L AWP", "BLUE", pointLeftRingsSafe);
  addAuton(355, 180, 100, 100, "Right AWP", "BLUE", pointRightRingsSafe);
  addAuton(455, 180, 100, 100, "Right Elim", "BLUE", pointRightRingRushElim);

  addAuton(40, 180, 100, 100, "Left Elim", "RED", pointGoalSideLeft);
  addAuton(140, 180, 100, 100, "Left AWP", "RED", pointLeftRingsSafe);
  addAuton(300, 180, 100, 100, "R AWP", "RED", pointRightRingsSafe);
  addAuton(390, 155, 100, 50, "R ElimWS", "RED", pointRightGoalElimWall);
  addAuton(390, 205, 100, 50, "R Elim", "RED", pointGoalSideRight);
  int numAutos = 5;


  //lcdButton skills = lcdButton(210, 180, 240, 100, "SKILLS", greenHex, false);


  std::string selectedColor = "GREEN";
  bool moveRelease = true;
  bool chosenColor = false;

  bool selectRelease = true;
  lcdButton tmColor = lcdButton(240, 100, 500, 300, "", redHex);

  bool haveDrawField = false;

  int currentAuto = 1;
  

  std::string writtenAuto = "NULL";

  while(runningProgram == false){

    if(move.value()<0.5 && moveRelease==true){
      moveRelease = false;
      if(chosenColor==false){
        if(selectedColor=="GREEN"){
          selectedColor="RED";
        }
        else if(selectedColor=="RED"){
          selectedColor="BLUE";
        }
        else if(selectedColor=="BLUE"){
          selectedColor= "GREEN";
        }
      }
      else{
        auton = -1;
        currentAuto++;
        if(currentAuto>numAutos){
          currentAuto = 1;
        }

      }
    }
    if(move.value()>0.5){
      moveRelease = true;
    }

    if(chosenColor == false){
      if(selectedColor=="RED")
          tmColor.setColor(redHex);
          //Brain.Screen.drawImageFromFile("red.jpg", 0, 0);
      else if(selectedColor=="BLUE")
          tmColor.setColor(blueHex);
          //Brain.Screen.drawImageFromFile("blue.jpg", 0, 0);
      else if(selectedColor=="GREEN")
          tmColor.setColor(greenHex);
    }

    if(select.value()<0.5 && selectRelease==true){
      selectRelease = false;
      haveDrawField=false;

      if(chosenColor==false){
        chosenColor=true;
        tmColor.setVisible(false);

        //IMPORTANT
        teamColor = selectedColor;
      }
      else{
        if(teamColor=="BLUE"){
          int counter = 1;
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "BLUE"){
              counter++;
            }
            if(counter == currentAuto){
              auton = autonButtons[i].name + autonButtons[i].team;
            }
          }
        }
        else if(teamColor == "RED"){
          int counter = 0;
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "RED"){
              counter++;
            }
            if(counter == currentAuto){
              auton = autonButtons[i].name + autonButtons[i].team;
            }
          }
        }
      }
    }

    if(select.value()>0.5){
      selectRelease = true;
    }

    if(chosenColor==true){
      if(teamColor=="RED"){
        if(haveDrawField==false){
          Brain.Screen.drawImageFromFile("/VEX/redSelect.png", 0, 0);
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "RED"){
              autonButtons[i].linkedButton.setVisible(true);
            }
          }
        }
        
      }
      else if(teamColor=="BLUE"){
        if(haveDrawField==false){
          Brain.Screen.drawImageFromFile("/VEX/blueSelect.png", 0, 0);
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "BLUE"){
              autonButtons[i].linkedButton.setVisible(true);
            }
          }
        }
      }
      else if(teamColor=="GREEN"){
        if(haveDrawField==false){
          Brain.Screen.drawImageFromFile("/VEX/greenSelect.png", 0, 0);
          //blueLeft.setVisible(true);
          //blueRight.setVisible(true);
          //skills.setVisible(true);
        }
      }
    }

    
    for(int i = 0; i<autoCounter; i++){
      if(autonButtons[i].linkedButton.pressing()){
        auton = autonButtons[i].name + autonButtons[i].team;
      }
    }

    if(haveDrawField==false){

      std::string tempSelect = "NULL";

      if(teamColor=="BLUE"){
          int counter = 0;
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "BLUE"){
              counter++;
            }
            if(counter == currentAuto){
              tempSelect = "TEMP" + autonButtons[i].name + autonButtons[i].team ;
            }
          }
        }
        else if(teamColor == "RED"){
          int counter = 0;
          for(int i = 0; i<autoCounter; i++){
            if(autonButtons[i].team == "RED"){
              counter++;
            }
            if(counter == currentAuto){
              tempSelect = "TEMP" + autonButtons[i].name + autonButtons[i].team;
            }
          }
        }


      int counter = 0;
      for(int i = 0; i<autoCounter; i++){
        if(auton == autonButtons[i].name + autonButtons[i].team){
          autonButtons[i].linkedButton.setColor(greenHex);
        }
        else if(tempSelect == "TEMP" + autonButtons[i].name + autonButtons[i].team){
          autonButtons[i].linkedButton.setColor(yellowHex);
        }
        else {
          if(autonButtons[i].team=="RED"){
            autonButtons[i].linkedButton.setColor(redHex);
          }
          else if(autonButtons[i].team=="BLUE"){
            autonButtons[i].linkedButton.setColor(blueHex);
          }
          //redRightElim.setColor(yellowHex);
          
        }
        // }if(currentAuto == )
      }

    }
    


    // if(writtenAuto!=auton){
    //   if(auton!="NULL"){
    //     writeFile("/VEX/auton.txt", auton);
    //     writeFile("/VEX/teamColor.txt", teamColor);
    //   }
    // }
  }
  return 0;
}


// int selector(){

//   lcdButton blueLeftElim = lcdButton(80, 180, 100, 100, "L Elims", blueHex, false);
//   lcdButton blueLeftElimWall = lcdButton(170, 205, 100, 50, "L ElimsTS", blueHex, false);
//   lcdButton blueLeftAWP = lcdButton(170, 155, 100, 50, "L AWP", blueHex, false);
//   lcdButton blueRightAWP = lcdButton(355, 180, 70, 100, "Right AWP", blueHex, false);
//   lcdButton blueRightElim = lcdButton(425, 180, 70, 100, "Right Elim", blueHex, false);


//   lcdButton redLeftElim = lcdButton(45, 180, 70, 100, "Left Elim", redHex, false);
//   lcdButton redLeftAWP = lcdButton(105, 180, 70, 100, "Left AWP", redHex, false);
//   lcdButton redRightAWP = lcdButton(300, 155, 100, 50, "R AWP", redHex, false);
//   lcdButton redRightElimWall = lcdButton(390, 205, 100, 50, "R ElimTS", redHex, false);
//   lcdButton redRightElim = lcdButton(390, 180, 100, 100, "R Elim", redHex, false);



//   lcdButton skills = lcdButton(210, 180, 240, 100, "SKILLS", greenHex, false);


//   std::string selectedColor = "GREEN";
//   bool moveRelease = true;
//   bool chosenColor = false;

//   bool selectRelease = true;
//   lcdButton tmColor = lcdButton(240, 100, 500, 300, "", redHex);

//   bool haveDrawField = false;

//   int currentAuto = 1;
//   int numAutos = 3;

//   std::string writtenAuto = "NULL";

//   while(runningProgram == false){

//     if(move.value()<0.5 && moveRelease==true){
//       moveRelease = false;
//       if(chosenColor==false){
//         if(selectedColor=="GREEN"){
//           selectedColor="RED";
//         }
//         else if(selectedColor=="RED"){
//           selectedColor="BLUE";
//         }
//         else if(selectedColor=="BLUE"){
//           selectedColor= "GREEN";
//         }
//       }
//       else{
//         auton = "NULL";
//         currentAuto++;
//         if(currentAuto>numAutos){
//           currentAuto = 1;
//         }

//       }
//     }
//     if(move.value()>0.5){
//       moveRelease = true;
//     }

//     if(chosenColor == false){
//       if(selectedColor=="RED")
//           tmColor.setColor(redHex);
//           //Brain.Screen.drawImageFromFile("red.jpg", 0, 0);
//       else if(selectedColor=="BLUE")
//           tmColor.setColor(blueHex);
//           //Brain.Screen.drawImageFromFile("blue.jpg", 0, 0);
//       else if(selectedColor=="GREEN")
//           tmColor.setColor(greenHex);
//     }

//     if(select.value()<0.5 && selectRelease==true){
//       selectRelease = false;
//       haveDrawField=false;

//       if(chosenColor==false){
//         chosenColor=true;
//         tmColor.setVisible(false);

//         //IMPORTANT
//         teamColor = selectedColor;
//       }
//       else{
//         if(teamColor=="BLUE"){
//           if(currentAuto == 1){
//             auton = "BlueLeftElim";
//           }
//           if(currentAuto == 2){
//             auton = "BlueLeftElim";
//           }
//           else if(currentAuto == 3){
//             auton = "BlueLeftAWP";
//           }
//           else if(currentAuto == 3){
//             auton = "BlueRightAWP";
//           }
//         }
//         else if(teamColor == "RED"){
//           if(currentAuto == 1){
//             auton = "RedLeftAWP";
//           }
//           else if(currentAuto == 2){
//             auton = "RedRightAWP";
//           }
//           else if(currentAuto == 3){
//             auton = "RedRightElim";
//           }
//         }
//       }
//     }
//     if(select.value()>0.5){
//       selectRelease = true;
//     }

//     if(chosenColor==true){
//       if(teamColor=="RED"){
//         if(haveDrawField==false){
//           Brain.Screen.drawImageFromFile("/VEX/redSelect.png", 0, 0);
//           redLeftElim.setVisible(true);
//           redLeftAWP.setVisible(true);
//           redRightAWP.setVisible(true);
//           redRightElimWall.setVisible(true);
//           redRightElim.setVisible(true);
//         }
        
//       }
//       else if(teamColor=="BLUE"){
//         if(haveDrawField==false){
//           Brain.Screen.drawImageFromFile("/VEX/blueSelect.png", 0, 0);
//           blueLeftElim.setVisible(true);
//           blueLeftElimWall.setVisible(true);
//           blueLeftAWP.setVisible(true);
//           blueRightAWP.setVisible(true);
//           blueRightElim.setVisible(true);
//         }
//       }
//       else if(teamColor=="GREEN"){
//         if(haveDrawField==false){
//           Brain.Screen.drawImageFromFile("/VEX/greenSelect.png", 0, 0);
//           //blueLeft.setVisible(true);
//           //blueRight.setVisible(true);
//           skills.setVisible(true);
//         }
//       }
//     }

    
//     if(blueLeftElim.pressing()) auton = "BlueLeftElim";
//     if(blueLeftElimWall.pressing()) auton = "BlueLeftElimWall";
//     if(blueLeftAWP.pressing()) auton = "BlueLeftAWP";
//     if(blueRightAWP.pressing()) auton = "BlueRightAWP";
//     if(blueRightElim.pressing()) auton = "BlueRightElim";

//     if(redLeftElim.pressing()) auton = "RedLeftElim";
//     if(redLeftAWP.pressing()) auton = "RedLeftAWP";
//     if(redRightAWP.pressing()) auton = "RedRightAWP";
//     if(redRightElimWall.pressing()) auton = "RedRightElimWall";
//     if(redRightElim.pressing()) auton = "RedRightElim";


//     if(skills.pressing()) auton = "Skills";

//     if(haveDrawField==false){

    

//       if(auton == "BlueLeftElim")
//         blueLeftElim.setColor(greenHex);
//       else if(currentAuto==1)
//         blueLeftElim.setColor(yellowHex);
//       else
//         blueLeftElim.setColor(blueHex);
      
//       if(auton == "BlueLeftElimWall")
//         blueLeftElimWall.setColor(greenHex);
//       else if(currentAuto==1)
//         blueLeftElimWall.setColor(yellowHex);
//       else
//         blueLeftElimWall.setColor(blueHex);

//       if(auton == "BlueLeftAWP")
//         blueLeftAWP.setColor(greenHex);
//       else if(currentAuto==2)
//         blueLeftAWP.setColor(yellowHex);
//       else
//         blueLeftAWP.setColor(blueHex);

//       if(auton == "BlueRightAWP")
//         blueRightAWP.setColor(greenHex);
//       else if(currentAuto==3)
//         blueRightAWP.setColor(yellowHex);
//       else
//         blueRightAWP.setColor(blueHex);

//       if(auton == "BlueRightElim")
//         blueRightElim.setColor(greenHex);
//       else if(currentAuto==3)
//         blueRightElim.setColor(yellowHex);
//       else
//         blueRightElim.setColor(blueHex);



//       if(auton == "RedLeftElim")
//         redLeftElim.setColor(greenHex);
//       else if(currentAuto==1)
//         redLeftElim.setColor(yellowHex);
//       else
//         redLeftElim.setColor(redHex);

//       if(auton == "RedLeftAWP")
//         redLeftAWP.setColor(greenHex);
//       else if(currentAuto==1)
//         redLeftAWP.setColor(yellowHex);
//       else
//         redLeftAWP.setColor(redHex);

//       if(auton == "RedRightAWP")
//         redRightAWP.setColor(greenHex);
//       else if(currentAuto==2)
//         redRightAWP.setColor(yellowHex);
//       else
//         redRightAWP.setColor(redHex);

//       if(auton == "RedRightElimWall")
//         redRightElimWall.setColor(greenHex);
//       else if(currentAuto==3)
//         redRightElimWall.setColor(yellowHex);
//       else
//         redRightElimWall.setColor(redHex);
      
//       if(auton == "RedRightElim")
//         redRightElim.setColor(greenHex);
//       else if(currentAuto==3)
//         redRightElim.setColor(yellowHex);
//       else
//         redRightElim.setColor(redHex);

      
      




//       if(auton == "Skills")
//         skills.setColor(greenHex);
//       else if(currentAuto==2)
//         skills.setColor(yellowHex);
//       else
//         skills.setColor(redHex);
      

//     }
    


//     // if(writtenAuto!=auton){
//     //   if(auton!="NULL"){
//     //     writeFile("/VEX/auton.txt", auton);
//     //     writeFile("/VEX/teamColor.txt", teamColor);
//     //   }
//     // }
//   }
//   return 0;
// }



void siberianCryotube(){
  // turnTest();
  // wait(50,sec);

  
  //logD("RUN AUTONOMOUS RUNNER", "", true);
  //pureTest();

  runningProgram = true;

  // if(auton=="NULL"){
  //   auton = readFile("/VEX/auton.txt");
  // }
  // if(teamColor=="NULL"){
  //   teamColor = readFile("/VEX/teamColor.txt");
  // }



  chassis.set_coordinates(0, 0, 0);



  

  //FOR LEFT SIDE
  //pointLeftRings();



  teamColor="RED";

  //Safe AWP Left Side
  //pointLeftRingsSafe();


  //Sade AWP Right Side
  pointRightRingsSafe();


  //pointRightGoalElimWall();

  
//testColorSort();
   

   //ELIMS

   //pointLeftRingRushElim();
   //pointRightRingRushElim();

   //BLUE
   //pointGoalSideLeft();

   //RED
  //pointGoalSideRight();


  //Skills 
  //pointSkills();
  //pointSkills();



  wait(500,sec);

  for(int i = 0; i<autoCounter; i++){
    if(auton == autonButtons[i].name + autonButtons[i].team){
      autonButtons[i].callback();
    }
  }

  /*

  if(auton == "BlueLeftElimWall") pointLeftGoalElimWall();
  if(auton == "BlueLeftElim") pointGoalSideLeft();
  if(auton == "BlueLeftAWP") pointLeftRingsSafe();

  if(auton == "BlueRightAWP") pointRightRingsSafe();
  if(auton == "BlueRightElim") pointRightRingRushElim();



  if(auton == "RedLeftElim") pointLeftRingRushElim();
  if(auton == "RedLeftAWP") pointLeftRingsSafe();

  if(auton == "RedRightAWP") pointRightRingsSafe();
  if(auton == "RedRightElimWall") pointRightGoalElimWall();
  if(auton == "RedRightElim") pointGoalSideRight();



  if(auton == "Skills") pointSkills();*/

}

//
// Main will set up the competition functions and callbacks.
//


int main() {
  //userControl();
  // logD("START PROGRAM", "main", true);


  // //this function is shit
  // displayInfo(Controller1, "Starting", "Pray this works", "eskjfeshf");
  // wait(0.5,seconds);

  // chassis.set_coordinates(0, 0, 0);
  setup();
  
  if(botLabMode == true){
    //setup();
    botLab();
  }

teamColor="RED";

//testSensors();

  vex::task flatEarth = vex::task(selector);


  Competition.drivercontrol(userControl);
  Competition.autonomous(siberianCryotube);

  

//Competition.drivercontrol(realUserControl);
//Competition.drivercontrol(akshayUC);
//Competition.autonomous(workingAWP);

  while (true) {
    wait(100, msec);
  }
}