#include "botSetup/vex.h"

#include <cmath>

#include <string>

#include "info/tuneAble.h"

#include "info/settings.h" 

#include "Libraries/controllerUI.h"

#include "Libraries/stringFunctions.h"

#include "Libraries/botErrorProcesses.h"

#include "DriveFuncs/motorFuncs.h" 
  
#include "DriveFuncs/PSO_Turn.h"

#include "botSetup/robot-config.h"

#include "info/log.h" 

#include "autos/tests.h"

std::string selector = "";
std::string lastText3 = "";

struct motorSetup { 
    std::string name;
    int32_t port; 
    vex::motor motorObject = motor(PORT21);//Default
};  

void testMotors(){
  while(true){
    vex::motor userMotor = Intake;
    motorSetup userPort;
    wait(0.4,seconds);
    selector = controllerSelector(Controller1, "Left Drivetrain", "Right Drivetrain", "Intake/Fly");
    
    if(selector == "EXIT"){
      return;
    }
    
    if(selector == "Left Drivetrain"){
      selector = controllerSelector(Controller1, "Front", "Middle", "Back");
      if(selector == "Front"){
        userMotor = LeftFrontMotor;
        userPort = motors[0];
      }
      if(selector == "Middle"){
        userMotor = LeftMiddleMotor;
        userPort = motors[1];
      }
      if(selector == "Back"){
        userMotor = LeftBackMotor;
        userPort = motors[2];
      }
    }

    if(selector == "Right Drivetrain"){
      selector = controllerSelector(Controller1, "Front", "Middle", "Back");
      if(selector == "Front"){
        userMotor = RightFrontMotor;
        userPort = motors[3];
      }
      if(selector == "Middle"){
        userMotor = RightMiddleMotor;
        userPort = motors[4];
      }
      if(selector == "Back"){
        userMotor = RightBackMotor;
        userPort = motors[5];
      }
    }

    if(selector == "Intake/Fly"){
      selector = controllerSelector(Controller1, "Intake", "Flywheel");
      if(selector == "Intake"){
        userMotor = Intake;
        userPort = motors[8];
      }
      if(selector == "Fly"){

      }
    }

    LeftMotors.setStopping(coast);
    RightMotors.setStopping(coast);

    int counter = 100;

    while(!Controller1.ButtonB.pressing()){
      if(userMotor.installed() == true){
        std::string powerString = "Power:" + floatToString( int(userMotor.voltage()*100)/100 ) + "*" + floatToString(int(userMotor.current()*100)/100 );
        if(counter>=100){
          displayInfo(Controller1, userPort.name + " in Port " + floatToString(userPort.port+1), "Temp:" + floatToString(userMotor.temperature(celsius)) + " " "RPM:" + floatToString(userMotor.velocity(rpm)), powerString);
          counter = 0;
        }
      }
      else{
        if(counter>=100){
          displayInfo(Controller1, userPort.name, "Motor Disconnected", "Port:" + floatToString(userPort.port+1));
          counter = 0;
        }
      }
      counter++;

      float velocity = Controller1.Axis3.value() + Controller1.Axis2.value();
      userMotor.spin(forward);
      userMotor.setVelocity(velocity, pct);

      wait(10, msec);
    }

    wait(0.25,seconds);
    
  }
}





void PIDlab(){

  //Default values
  float dmaxVoltage = 12;
  float dp = .4;
  float di = .03;
  float dd = 3;
  float dstarti = 15;

  //real values
  double maxVoltage = dmaxVoltage;
  double kp = dp;
  double ki = di;
  double kd = dd;
  double starti = dstarti;

  bool editValues = true;
  bool doTurn = true;//will always be true

  std::string popData = "";
  std::string fileName = "NONE";
  int num_particles = 5;

  selector = controllerSelector(Controller1, "Test Parameters", "Modify Profile", "EXIT");
  if(selector == "EXIT"){
    return;
  }
  if(selector == "Test Parameters"){
    while(true){
    int value = 0;

    while(editValues){
      wait(0.2,seconds);
      std::string line1, line2;
      //This is the speed that the value is going to be vhanged by
      float velocity = (Controller1.ButtonUp.pressing() - Controller1.ButtonDown.pressing())/100;
      velocity += deadband( Controller1.Axis2.value() + Controller1.Axis3.value(), 8)/(150*7);
      
      if(value == 0){
        maxVoltage += velocity/2;
        line1 = "MaxVolts: " + floatToString(maxVoltage);
        line2 = "Default: " + floatToString(dmaxVoltage);
      }
      if(value == 1){
        kp += velocity/6;
        line1 = "Kp: " + floatToString(kp);
        line2 = "Default: " + floatToString(dp);
      }
      if(value == 2){
        ki += velocity/100;
        line1 = "Ki: " + floatToString(ki);
        line2 = "Default: " + floatToString(di);
      }
      if(value == 3){
        kd += velocity/1.5;
        line1 = "Kd: " + floatToString(kd);
        line2 = "Default: " + floatToString(dd);
      }
      if(value == 4){
        starti += velocity;
        line1 = "StartI: " + floatToString(starti);
        line2 = "Default: " + floatToString(dstarti);
      }
      
      displayInfo(Controller1, line1, line2, "L/R Arrows and A");

      if(Controller1.ButtonLeft.pressing()){
        value -= 1;
        if(value<0){
          value = 4;
        }
      }
      if(Controller1.ButtonRight.pressing()){
        value += 1;
        if(value>4){
          value = 0;
        }
      }

      if(Controller1.ButtonB.pressing()){
        return;
      }
      if(Controller1.ButtonA.pressing()){
        editValues = false;
      }

      wait(10, msec);
    }

    if(doTurn){
    chassis.set_turn_constants(maxVoltage, kp, ki, kd, starti);
    selector = controllerSelector(Controller1, "45 Degree", "90 Degree", "180 Degree");

    int startTime = Brain.timer(msec);
    float error = 0;
    chassis.set_heading(0);
    if(selector == "45 Degree"){
      chassis.turn_to_angle(45);
      error = chassis.get_absolute_heading()-45;
    }
    if(selector == "90 Degree"){
      chassis.turn_to_angle(90);
      error = chassis.get_absolute_heading()-90;
    }
    if(selector == "180 Degree"){
      chassis.turn_to_angle(180);
      error = chassis.get_absolute_heading()-180;
    }

    int finalTime = Brain.timer(msec) - startTime;
    displayInfo(Controller1, selector + " Turn", "Time: " + floatToString(finalTime), "Error: " + floatToString(error));
    wait(1.0, seconds);
    }

    selector = controllerSelector(Controller1, "Edit Values", "Run Turn", "EXIT");
    if(selector == "EXIT"){
      return;
    }
    if(selector == "Edit Values"){
      editValues = true;
      doTurn = true;
    }
    if(selector == "Run Turn"){
      editValues = false;
      doTurn = true;
    }

    }

  }


  if(selector == "Modify Profile"){
    std::string type = controllerSelector(Controller1, "DRIVE", "TURN", "SWING");
    std::string angles[] = {"12", "30", "45", "90", "135", "180"};
    std::string distances[] = {"6", "10", "15", "24", "40", "60"};
    //PSOTy = type;
    PSOTy = type;

    while(true){
      
    std::string bot = controllerSelector(Controller1, "Real_Bot", "Test_Bot", "Other");
    std::string selectedAngle = "";
    if(type=="DRIVE"){
      selectedAngle = controllerSelectorEXT(Controller1, 6, distances);
    }
    else{
      selectedAngle = controllerSelectorEXT(Controller1, 6, angles);
    }
    
    //, "45", "90", "135", "180"});
    // std::string selectedAngle = controllerSelector(Controller1, "30", "45", "OTHER");
    // if(selectedAngle == "OTHER"){
    //   selectedAngle = controllerSelector(Controller1, "90", "135", "180");
    // }
    

    fileName = "/VEX/PSO_Profiles/" + bot + "/" + type + "/" + selectedAngle + "_degrees.txt"; 

    selector = controllerSelector(Controller1, "LOAD", "RUN", "NEW");
    if(selector == "NEW"){

      popData = getPSOPopulation(num_particles, STI(selectedAngle));
      if(Brain.SDcard.exists(fileName.c_str()) == true){

        selector = confirmControllerSelector(Controller1, "Overwrite?", "Dont do it");
        if(selector == "Overwrite?"){
          writeFile(fileName, popData);
        }
      }
      else{
        writeFile(fileName, popData);
      }
      displayInfo(Controller1, "Done, Returning");
      wait(0.2,seconds);
      
    }
    if(selector == "RUN"){
    //   std::string selectedAngle = controllerSelector(Controller1, "30", "45", "OTHER");
    // if(selectedAngle == "OTHER"){
    //   selectedAngle = controllerSelector(Controller1, "90", "135", "180");
    // }

      int pos = popData.find(",");
    	int popAngle = STI(popData.substr(0, pos));

      if(STI(selectedAngle) == popAngle){
        PSOLoop(20, num_particles, popData, popAngle,  0.1, 1, 2, fileName);
      }
      else{
        controllerSelector(Controller1, "The angle", "loaded doesn't", "equal angle entered");
      }

    }
    if(selector == "LOAD"){
      

      fileName = "/VEX/PSO_Profiles/" + bot + "/" +selectedAngle + "_degrees.txt";

      if(Brain.SDcard.exists(fileName.c_str()) == true){
        
        displayInfo(Controller1, "Loaded", fileName);
        wait(0.7,seconds);
        popData = readFile(fileName);
        logD("LOG PSO LOADER", "Loaded file: " + fileName +", content:" + popData);
      }
      else{
        displayInfo(Controller1, "Error", "File DNE");
        logD("ERROR", "PSO FILE DNE");
        wait(1.5,seconds);
      } 
      displayInfo(Controller1, "Done, Returning");
      wait(0.2,seconds);
    }
    if(selector == "EXIT"){
      return;
    }
  }


  }

  if(selector == "Run New PSO"){
    std::string selectedAngle = controllerSelector(Controller1, "45", "90", "OTHER");
      if(selectedAngle == "OTHER"){
        selectedAngle = controllerSelector(Controller1, "135", "180");
      }
      popData = getPSOPopulation(num_particles, STI(selectedAngle));
      PSOLoop(20, num_particles, popData, STI(selectedAngle),  0.1, 1, 2, "NONE");
  }
}







void fastTestDrive(){

          controllerSelector(Controller1,"WHEELS WILL SPIN", "HOLD IN PLACE", "FOR LIST OF ERRORS");
          vex::motor userMotor = motor(PORT21);
          motorSetup userPort;
          std::string problemMotorsL = "";
          std::string problemMotorsR = "";
          
          for(int i = 0; i<6; i++){
            userPort = motors[i];
            userMotor = motors[i].motorObject;
            userMotor.setVelocity(100, pct);
            userMotor.spin(forward); 
            logD("TEST DRIVE", "Motor: " + userPort.name, true);

          for(int g = 0; g<1; g++){
            if(userMotor.installed() == true){
              std::string powerString = "Power:" + floatToString( int(userMotor.voltage()*100)/100 ) + "*" + floatToString(int(userMotor.current()*100)/100 );
              displayInfo(Controller1, userPort.name + " in Port " + floatToString(userPort.port+1), "Temp:" + floatToString(userMotor.temperature(celsius)) + " " "RPM:" + floatToString(userMotor.velocity(rpm)), powerString);
            }
            else{
              displayInfo(Controller1, userPort.name, "Motor Disconnected", "Port:" + floatToString(userPort.port+1));
            }
            if(Controller1.ButtonB.pressing()){
              return;
            }
            wait(200,msec);
          }

          if(fabs(userMotor.velocity(pct))>15){//THe motor is spinning
            logD("ERROR MOTOR ENAGEMENT", "The motor:" + userPort.name + ", in port:" + FTS(userPort.port + 1) + ", is not engaged", true);
            if(userPort.name.substr(0,1) == "L"){
              problemMotorsL += userPort.name + " ";
            }
            if(userPort.name.substr(0,1) == "R"){
              problemMotorsR += userPort.name + " ";
            }
          
          }

          userMotor.setVelocity(-100, pct);

          for(int g = 0; g<1; g++){
            if(userMotor.installed() == true){
              std::string powerString = "Power:" + floatToString( int(userMotor.voltage()*100)/100 ) + "*" + floatToString(int(userMotor.current()*100)/100 );
              displayInfo(Controller1, userPort.name + " in Port " + floatToString(userPort.port+1), "Temp:" + floatToString(userMotor.temperature(celsius)) + " " "RPM:" + floatToString(userMotor.velocity(rpm)), powerString);
            }
            else{
              displayInfo(Controller1, userPort.name, "Motor Disconnected", "Port:" + floatToString(userPort.port+1));
            }
            if(Controller1.ButtonB.pressing()){
              return;
            }
            wait(200,msec);
          }

            setMotorVelocities(0);
          }

    displayInfo(Controller1, "Problem Motors:", problemMotorsR, problemMotorsL);

    while(Controller1.ButtonA.pressing() == false){wait(25, msec);}
    

}









void botLab(){
  //logD("RUN BOT_LAB", "Running botlab", true);
  while(true){
    selector = controllerSelector(Controller1, "Motor Lab", "PID Lab", "Drive/Auton");

    if(selector == "Motor Lab"){
      
      selector = controllerSelector(Controller1, "Test Motor", "Get Heatmap", "Test Drive");
      logD("RUN MOTOR_LAB", "Running motorlab mode: " + selector, true);
      if(selector == "Test Motor"){
        testMotors();
      }
      if(selector == "Get Heatmap"){
        lastText3 = "";

        while(!Controller1.ButtonB.pressing()){
          std::string firstLine;
          std::string secondLine;
          std::string thirdLine;
          firstLine = getHeat1();
          secondLine = getHeat2();
          thirdLine = getHeat3();
          std::string newText = firstLine + secondLine + thirdLine;
          if (lastText3 != newText) {
            displayInfo(Controller1, firstLine, secondLine, thirdLine);
          }
          lastText3 = newText;
          }

        }


        if(selector == "Test Drive"){
          fastTestDrive();
        }
        
    }


    else if(selector == "PID Lab"){
      PIDlab();
    }
    else if(selector == "Drive/Auton"){
      selector = controllerSelector(Controller1, "UserControl", "Autons");
      if(selector=="UserControl"){
        displayInfo(Controller1, "RUNNING BASIC", "USERCONTROL", "TEMPS IGNORED");
        logD("RUN BASIC_USER_CONTROL", "Running basic usercontrol, temps ignored");
        while(1){
          float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
          float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
          LeftMotors.spin(fwd, to_volt(leftthrottle), volt);
          RightMotors.spin(fwd, to_volt(rightthrottle), volt);
          if(Controller1.ButtonB.pressing()){
            return;
          }

          if(Controller1.ButtonR1.pressing()){
            Intake.spin(forward);
            Intake.setVelocity(100, percent);
          }
          else if(Controller1.ButtonR2.pressing()){
            Intake.spin(forward);
            Intake.setVelocity(-100, percent);
          }
          else{
            Intake.setVelocity(0, percent);
          }
          wait(10,msec);
        }
      }

      
      if(selector == "Autons"){
        selector = controllerSelector(Controller1, "Autons", "Tests");
        if(selector=="Tests"){
          std::string programs[] = {"odom_test()", "pure_test()", "dist_test()", "pointTurns()", "regTurns()", "accelTest()"};
          selector = controllerSelectorEXT(Controller1, 6, programs);
          if(selector=="odom_test()"){
            odom_test();
          }
          else if(selector=="pure_test()"){
            pureTest();
          }
          else if(selector=="dist_test()"){
            distanceTest();
          }
          else if(selector=="pointTurns()"){
            turnTestPoint();
          }
          else if(selector=="regTurns()"){
            turnTestReg();
          }
          else if(selector=="accelTest()")
          {
            accelTest();
          }

          


        }
        if(selector=="Autons"){
          if(selector=="autonOne()"){
            autonOne();
          }
          if(selector=="autonTwo()"){
            autonTwo();
          }
        }
      }
      return;
    }

  }
  botLab();
} 