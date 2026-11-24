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

#include "Sensing/distance.h"

#include "DriveFuncs/motorFuncs.h" 



double limitVoltageForSlip(double desiredVolts, double currentVolts) {
    const double MAX_VOLTS = 12.0;
    //const double WHEEL_RADIUS = 0.051;  // meters (4" wheels)
    const double ROBOT_MASS = 5.0;      // kg
    const double FRICTION_COEF = 0.7;   // rubber on field
    
    // Calculate max voltage before slip based on current speed
    double maxSafeVolts = (currentVolts > 0) 
        ? currentVolts * FRICTION_COEF * ROBOT_MASS / 2.0
        : MAX_VOLTS * 0.7;  // 70% of max when starting
    
    return std::min(desiredVolts, maxSafeVolts);
}

//Linear voltage change in the drivetrain motors
double linearAccel(double desiredVolts, double currentVolts) {
  //Constant value that needs to be tuned
  double MxAccel = 3;
  
  //If the currentVolts is not tracked by the program, it can easily be found
  if(currentVolts == -100){
    currentVolts = (LeftMotors.voltage() + RightMotors.voltage())/2;
  }

  //Generate the output voltage and return the value for use
  float output = clamp(desiredVolts, currentVolts-(MxAccel/1), MxAccel+currentVolts);
  return output;
}

//logD("LOG LINEAR", "DesiredVolts:" + FTS(desiredVolts) + ", CurrentVolts:" + FTS(currentVolts) + ", OutputVolts:" + FTS(output));
  


float forceAcceleration(float target){
  double safeLeftVolts = linearAccel(target, LeftMotors.voltage());
  double safeRightVolts = linearAccel(target, RightMotors.voltage());
    
  LeftMotors.spin(vex::forward, safeLeftVolts, vex::volt);
  RightMotors.spin(vex::forward, safeRightVolts, vex::volt);


  return safeLeftVolts;
}

void testSlip(){
  logD("START TESTSLIP", "Test wheel slip for driving forwards", true);
  float output = 0;
  LeftMotors.resetPosition();
  RightMotors.resetPosition();

  rotation RTemp = rotation(PORT10);

  for(int i = 0; i<1000; i+=10){
    logD("STEP TESTSLIP", "Time:" + FTS(i) + ", MotorVel:" + FTS(output) + ", LMS-Deg:" + FTS(LeftMotors.position(degrees)) + ", RMS-Deg:" + FTS(RightMotors.position(degrees)) + ", RTemp-Deg:" + FTS(RTemp.position(degrees)));

    if(i<450){
      forceAcceleration(-12);
      //setMotorVelocities(100);
      output = 100;
    }
    else{
      forceAcceleration(12);
      //setMotorVelocities(-100);
      output = -100;
    }

    wait(10, msec);
  }

  logD("STOP TESTSLIP", "STOP Test wheel slip for driving forwards", true);

}



void RUSH(float desiredHeading){
  PID headingPID(reduce_negative_180_to_180(desiredHeading - chassis.get_absolute_heading()), chassis.heading_kp, chassis.heading_ki, chassis.heading_kd, chassis.heading_starti);
  
  float heading_error = reduce_negative_180_to_180(desiredHeading - chassis.get_absolute_heading());
  float heading_output = headingPID.compute(heading_error);

  heading_output = clamp(heading_output, -chassis.heading_max_voltage, chassis.heading_max_voltage);
  float drive_output = 12;

  chassis.drive_with_voltage(drive_output+(2*heading_output), drive_output-(2*heading_output));
}



float slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}