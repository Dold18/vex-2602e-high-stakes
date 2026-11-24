#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "info/log.h"
#include "JAR-Template/PID.h"
#include "Sensing/slip.h"
#include "autos/callbacks.h"

#include <string>
#include <array>
#include <random>
#include <functional>
 

void call4THC(float angle, int excTime, std::function<void()> callback){
  double error = reduce_negative_180_to_180(angle - chassis.get_absolute_heading());
  PID turnPID(error, chassis.turn_kp, chassis.turn_ki, chassis.turn_kd, chassis.turn_starti);

  logD("START PID CTURN","Current Angle:" + FTS(chassis.get_absolute_heading()) + ", Target Angle:" +  FTS(angle) + ", MaxVolt:" + FTS(chassis.turn_max_voltage) + ", KP:" + FTS(chassis.turn_kp) + ", KI:" + FTS(chassis.turn_ki) + ", KD:" + FTS(chassis.turn_kd) + ", StartI:" +FTS(chassis.turn_starti) + ", TSE:" +FTS(chassis.turn_settle_error) + ", TST:" +FTS(chassis.turn_settle_time) + ", Timeout:" + FTS(chassis.turn_timeout));
  bool hasBeenCalled = false;
  int counter = 0;
  // Desired heading carries over the angle from one movement to another. That way, if the robot doesn't
  // finish a turn movement, it will still drive at the angle that was specified in the turn movement.
  while(turnPID.is_settled() == false){
    if(counter>=excTime && hasBeenCalled == false){
      logD("RUNNING CALLBACK", "Running callback in call4THC at time:" + FTS(counter));
      hasBeenCalled = true;
      callback();
    }

    float error = reduce_negative_180_to_180(angle - chassis.get_absolute_heading());
    // Reducing the angle to a value between -180 and 180 degrees ensures that the robot always takes the 
    // shorter path when making a turn.
    float output = turnPID.compute(error);
    float mx = 0;
    output = clamp(output, -mx, mx);
    chassis.drive_with_voltage(output, -output);

    counter += 10;
    task::sleep(10);
  }
  chassis.DriveL.stop(hold);
  chassis.DriveR.stop(hold);


  logD("STOP PID CTURN", "");
}





void call4DD(float distance, double excTime, std::function<void()> callback, float chainDistBreak){
  //logD("START PID CDRIVE","distance:" +  FTS(distance) + ", MaxVolt:" + FTS(chassis.drive_max_voltage) + ", KP:" + FTS(chassis.drive_kd) + ", KI:" + FTS(chassis.drive_ki) + ", KD:" + FTS(chassis.drive_kd) + ", StartI:" +FTS(chassis.drive_starti) + ", TSE:" +FTS(chassis.drive_settle_error) + ", TST:" +FTS(chassis.drive_settle_time) + ", Timeout:" + FTS(chassis.drive_timeout));
  float heading = chassis.get_absolute_heading();
  //chassis.desired_heading = heading;
  PID drivePID(distance, chassis.drive_kp, chassis.drive_ki, chassis.drive_kd, chassis.drive_starti, chassis.drive_settle_error, chassis.drive_settle_time, chassis.drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - chassis.get_absolute_heading()), chassis.heading_kp, chassis.heading_ki, chassis.heading_kd, chassis.heading_starti);
  float start_average_position = (chassis.get_left_position_in()+chassis.get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at
  // and determines error relative to that value.
  float average_position = start_average_position;
  bool hasBeenCalled = false;
  int counter = 0;

  float pastVolt = 0;
  while(drivePID.is_settled() == false){
    

    average_position = (chassis.get_left_position_in()+chassis.get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;

    if(hasBeenCalled == false && fabs(drive_error)<=(fabs(distance)-excTime)){
      //logD("RUNNING CALLBACK", "Running callback in call4DD at time:" + FTS(counter));
      hasBeenCalled = true;
      callback();
    }
    if(fabs(chainDistBreak)<=(fabs(distance)-excTime) && chainDistBreak!=-1){
      return;
    }

    float heading_error = reduce_negative_180_to_180(heading - chassis.get_absolute_heading());
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    float mv = chassis.drive_max_voltage;


    drive_output = linearAccel(drive_output, pastVolt);



    drive_output = clamp(drive_output, -mv, mv);
    heading_output = clamp(heading_output, -mv, mv);

    pastVolt = drive_output;
    //logD("LOG pastLogan","pastVolt:" + FTS(pastVolt));

    chassis.drive_with_voltage(drive_output+heading_output, drive_output-heading_output);

    counter += 10;
    wait(10, msec);
  }
  chassis.DriveL.stop(hold);
  chassis.DriveR.stop(hold);

  //logD("STOP PID CDRIVE", "");
}

void call4DD(float distance, double excTime, std::function<void()> callback){
  call4DD(distance, excTime, callback, -1);
}


void THC(float angle){
  call4THC(angle, 100000, blank);
}

void donaldDuck(float distance){
  call4DD(distance, 100000, blank, -1);
}

void donaldDuck(float distance, float chainBreak){
  call4DD(distance, 100000, blank, chainBreak);
}