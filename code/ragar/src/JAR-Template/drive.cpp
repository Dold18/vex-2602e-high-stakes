#include "botSetup/vex.h"
#include "info/settings.h"
#include "Libraries/stringFunctions.h"
#include "info/log.h"
#include "Sensing/slip.h"
#include "Libraries/ControllerUI.h"

Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale), 
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL), 
  DriveR(DriveR),
  //Every device below is passed by port rather than passing the object.
  //This ensures that the devices work properly.
  Gyro(inertial(gyro_port)), 
  DriveLF(DriveLF_port, is_reversed(DriveLF_port)),
  DriveRF(DriveRF_port, is_reversed(DriveRF_port)),
  DriveLB(DriveLB_port, is_reversed(DriveLB_port)),
  DriveRB(DriveRB_port, is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
  if (drive_setup != ZERO_TRACKER_NO_ODOM){
    if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      //odom.set_physical_distances(ForwardTracker_center_distance, 0);
      //Why would we get rid of this?
      // Setting the sideways distance to 0 essentially tells the robot that there is a sideways tracker
      // in the center of the robot that never moves. Even though the tracker isn't really there, the odom
      // still works fine.

      //Just use this 
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    } else {
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
  }
}

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

// All PID constants are passed as kP, kI, kD, and startI. The kP, kI, and kD are pretty standard,
// but startI keeps the integral value at 0 until the absolute value of the error is below startI.
// This prevents integral windup on bigger turns.

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

float Drive::getCenterDistance(){
  return ForwardTracker_center_distance;
}

// Settle error and settle time work together to check whether the desired position was achieved, but
// timeout is separate. The robot must stay within the settle error for the duration of the settle time 
// to be settled. If the duration of the movement reaches timeout without being settled, the robot
// gives up and goes to the next movement. 

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
  return( DriveR.position(deg)*drive_in_to_deg_ratio );
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  
  //logD("START PID TURN","Current Angle:" + FTS(get_absolute_heading()) + ", Target Angle:" +  FTS(angle) + ", MaxVolt:" + FTS(turn_max_voltage) + ", KP:" + FTS(turn_kp) + ", KI:" + FTS(turn_ki) + ", KD:" + FTS(turn_kd) + ", StartI:" +FTS(turn_starti) + ", TSE:" +FTS(turn_settle_error) + ", TST:" +FTS(turn_settle_time) + ", Timeout:" + FTS(turn_timeout));
  
  desired_heading = angle;
  // Desired heading carries over the angle from one movement to another. That way, if the robot doesn't
  // finish a turn movement, it will still drive at the angle that was specified in the turn movement.
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    // Reducing the angle to a value between -180 and 180 degrees ensures that the robot always takes the 
    // shorter path when making a turn.
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);


  //logD("STOP PID TURN", "");
} 


void Drive::turn_to_angleAntiO(float angle){
  turn_to_angleAntiO(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angleAntiO(float angle, float turn_max_voltage){
  turn_to_angleAntiO(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angleAntiO(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angleAntiO(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angleAntiO(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  
  //logD("START PID TURN","Current Angle:" + FTS(get_absolute_heading()) + ", Target Angle:" +  FTS(angle) + ", MaxVolt:" + FTS(turn_max_voltage) + ", KP:" + FTS(turn_kp) + ", KI:" + FTS(turn_ki) + ", KD:" + FTS(turn_kd) + ", StartI:" +FTS(turn_starti) + ", TSE:" +FTS(turn_settle_error) + ", TST:" +FTS(turn_settle_time) + ", Timeout:" + FTS(turn_timeout));
  
  desired_heading = angle;

  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);

  float previous_error = reduce_negative_180_to_180(angle - get_absolute_heading()); // Initialize previous error
  float output; // Stores the motor output power
  bool has_overshot = false; // Tracks if overshoot has occurred

  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading()); // Calculate current error
    
    // Detect sign change (overshoot)
    if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0)) {
      //logD("Overshoot Detected", "Cutting power by .2");
      has_overshot = true; // Mark overshoot as detected
    }
    
    
    output = turnPID.compute(error); // Compute PID output
    output = clamp(output, -turn_max_voltage, turn_max_voltage); // Clamp the output to max voltage
    if(has_overshot){
      output *= 0.9;
    }
    drive_with_voltage(output, -output); // Send power to the motors

    previous_error = error; // Update previous error for the next iteration
    task::sleep(10); // Small delay for loop execution
  }

  // Stop the motors with hold after the turn is complete or if overshot
  DriveL.stop(hold);
  DriveR.stop(hold);

  //logD("STOP PID TURN", "");
} 

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  //logD("START PID DRIVE","distance:" +  FTS(distance) + ", MaxVolt:" + FTS(drive_max_voltage) + ", KP:" + FTS(drive_kd) + ", KI:" + FTS(drive_ki) + ", KD:" + FTS(drive_kd) + ", StartI:" +FTS(drive_starti) + ", TSE:" +FTS(drive_settle_error) + ", TST:" +FTS(drive_settle_time) + ", Timeout:" + FTS(drive_timeout));
  
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  // Rather than resetting the drive position , this function just notes what the drive position started at
  // and determines error relative to that value.
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    // Just like for turns, reducing from -180 to 180 degrees ensures that the robot takes the 
    // quickest path to the desired heading.
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);

  logD("STOP PID DRIVE", "");
}

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    DriveL.spin(fwd, output, volt);
    //Only the left side of the drive turns, making this a "left swing".
    DriveR.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}
//Allows for the left motor to be set to a velocity
// void Drive::right_drive_to_angle(float angle, float h){
//   right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
// }

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -swing_max_voltage, swing_max_voltage);
    DriveR.spin(reverse, output, volt);
    //Only the right side of the drive turns, so this is a "right swing".
    DriveL.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}


//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 
//CHECK THIS CHECK THIS CHECK THIS 

//CHECK THIS CHECK THIS CHECK THIS 

float Drive::get_ForwardTracker_position(){
  //Just do this because we have a tracker
  //return (get_right_position_in());
  return((get_right_position_in()+get_left_position_in())/2);//R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  
  if (drive_setup==ZERO_TRACKER_ODOM){
    // For zero tracker odom, the right side of the drive becomes the tracker.
    return (get_right_position_in());//+get_left_position_in())/2;
  }
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    // This if-else just discriminates based on whether the sensor is an encoder or rotation sensor.
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

float Drive::get_SidewaysTracker_position(){
  return(0);
  //return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);

  
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    //return(0); we got trackers so dont need this this
    // These setups all pretend that there is a sideways tracker  in the center that just never moves.
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    task::sleep(5);
  }
}

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation(orientation_deg*gyro_scale/360.0, deg);
}



void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);
  odom_task = task(position_track_task);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}



void Drive::drive_to_pointLemLib(float X_position, float Y_position){
  drive_to_pointLemLib(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointLemLib(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_pointLemLib(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointLemLib(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pointLemLib(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointLemLib(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, 
                           float drive_settle_error, float drive_settle_time, float drive_timeout, 
                           float drive_kp, float drive_ki, float drive_kd, float drive_starti, 
                           float heading_kp, float heading_ki, float heading_kd, float heading_starti) {
    // Initialize PID controllers
    PID drivePID(hypot(X_position - get_X_position(), Y_position - get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
    PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position - get_X_position(), Y_position - get_Y_position())) - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);

    // Initialize variables for tracking errors and outputs
    float drive_error = hypot(X_position - get_X_position(), Y_position - get_Y_position());
    float prev_drive_output = 0;  // Previous drive output for slew
    float prev_heading_output = 0; // Previous heading output for slew
    bool close = false;  // Indicator for proximity to the target

    logD("START DTP", "Starting drive to point, error: " + FTS(drive_error));

    // Main loop (runs until PID settles)
    while (!drivePID.is_settled()){// && 0.08<fabs(cos(to_rad(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()))))  ) {
    drive_error = hypot(X_position - get_X_position(), Y_position - get_Y_position());

    if(drive_error<10){
      // Get current position
    float current_x = get_X_position();
    float current_y = get_Y_position();

    // Compute heading angle
    float heading_angle = to_rad(get_absolute_heading());

    // Compute the closest point on the robot's path (projected onto the goal line)
    float projected_x = current_x + cos(heading_angle) * (X_position - current_x);
    float projected_y = current_y + sin(heading_angle) * (Y_position - current_y);

    // Compute Euclidean distance from the closest projected point to the target
    float corrected_distance = hypot(X_position - projected_x, Y_position - projected_y);
    //displayInfo(Controller1, FTS(corrected_distance), FTS(drive_error));
      drive_error-=corrected_distance;
    }

    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    

    float heading_output = 0;

        // Scale drive output based on heading alignment
        if (drive_error < 6) {
          heading_output = 0;
          if(heading_scale_factor<0){
            drive_output*=-1;
          }
          
        }
        else{
          drive_output*=heading_scale_factor;

          heading_error = reduce_negative_90_to_90(heading_error);
          // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
          // to do so.
          heading_output = headingPID.compute(heading_error);
        }


        // Check if close enough to stop heading corrections
        

        // Clamp outputs to maximum values
        drive_output = clamp(drive_output, -fabs(heading_scale_factor) * drive_max_voltage, fabs(heading_scale_factor) * drive_max_voltage);
        heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

        // Apply slew rate for smooth transitions
        drive_output = slew(drive_output, prev_drive_output, 3);
        heading_output = slew(heading_output, prev_heading_output, 0);

        // Update previous outputs
        prev_drive_output = drive_output;
        prev_heading_output = heading_output;

        // Apply motor voltages
        drive_with_voltage((drive_output + heading_output), (drive_output - heading_output));

        // Log data for debugging
        logD("STEP DTP", "X: " + FTS(chassis.get_X_position()) + ", Y:" + FTS(chassis.get_Y_position()) +
             ", Heading:" + FTS(chassis.get_absolute_heading()) + ", TX:" + FTS(X_position) + ", TY:" + FTS(Y_position) +
             ", driveError:" + FTS(drive_error) + ", headingError:" + FTS(heading_error) +
             ", driveOutput:" + FTS(drive_output) + ", headingOutput:" + FTS(heading_output));

        // Delay to save resources
        task::sleep(10);
    }

    // Finalize motion
    desired_heading = get_absolute_heading();
    drive_with_voltage(0, 0);  // Stop the drivetrain
    logD("STOP DTP", "Stopping drive to point, error: " + FTS(drive_error));
}


void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  
  logD("START DTP", "Starting drive to point, error:" + FTS(drive_error));
  
  while(drivePID.is_settled() == false){
    drive_error = hypot(X_position-get_X_position(), Y_position-get_Y_position());
    // The drive error is just equal to the distance between the current and desired points.
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    // The scale factor slows the drive down the more it's facing away from the desired point,
    // and that way the heading correction has time to catch up.
    //Idk might want two
    heading_error = reduce_negative_90_to_90(heading_error);
    // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
    // to do so.
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }
    // This if statement prevents the heading correction from acting up after the robot gets close
    // to being settled.

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    drive_output = linearAccel(drive_output, 0);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage((drive_output+heading_output), (drive_output-heading_output));
    logD("STEP DTP", "X: " + FTS(chassis.get_X_position()) + ", Y:" + FTS(chassis.get_Y_position()) + ", Heading:" + FTS(chassis.get_absolute_heading()) + ", TX:" + FTS(X_position) + ", TY:" + FTS(Y_position) + ", driveError:" + FTS(drive_error) + ", headingError:" + FTS(heading_error) + ", driveOutput:" + FTS(drive_output) + ", headingOutput:" + FTS(heading_output));
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  logD("STOP DTP", "Stopping drive to point, error:" + FTS(drive_error));
}

void Drive::drive_to_pointPure(float X_position, float Y_position){
  drive_to_pointPure(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointPure(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_pointPure(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointPure(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pointPure(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pointPure(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  
  logD("START PURE", "Starting drive to point, error:" + FTS(drive_error));
  
  //while(drivePID.is_settled() == false){
    while(drive_error > 15){
    drive_error = hypot(X_position-get_X_position(), Y_position-get_Y_position());
    // The drive error is just equal to the distance between the current and desired points.
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    // The scale factor slows the drive down the more it's facing away from the desired point,
    // and that way the heading correction has time to catch up.

    //Idk might want two
    heading_error = reduce_negative_90_to_90(heading_error);
    // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
    // to do so.
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }
    // This if statement prevents the heading correction from acting up after the robot gets close
    // to being settled.

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    drive_output = linearAccel(drive_output, 0);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage((drive_output+heading_output), (drive_output-heading_output));
    logD("STEP PURE", "X: " + FTS(chassis.get_X_position()) + ", Y:" + FTS(chassis.get_Y_position()) + ", Heading:" + FTS(chassis.get_absolute_heading()) + ", TX:" + FTS(X_position) + ", TY:" + FTS(Y_position) + ", driveError:" + FTS(drive_error) + ", headingError:" + FTS(heading_error) + ", driveOutput:" + FTS(drive_output) + ", headingOutput:" + FTS(heading_output));
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  logD("STOP PURE", "Stopping drive to point, error:" + FTS(drive_error));
}


void Drive::drive_to_pointChain(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti, float lDist){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  
  logD("START PURE", "Starting drive to point, error:" + FTS(drive_error));
  
  //while(drivePID.is_settled() == false){
    while(drive_error > lDist){
    drive_error = hypot(X_position-get_X_position(), Y_position-get_Y_position());
    // The drive error is just equal to the distance between the current and desired points.
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    // The scale factor slows the drive down the more it's facing away from the desired point,
    // and that way the heading correction has time to catch up.

    //Idk might want two
    heading_error = reduce_negative_90_to_90(heading_error);
    // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
    // to do so.
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }
    // This if statement prevents the heading correction from acting up after the robot gets close
    // to being settled.

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    drive_output = linearAccel(drive_output, 0);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage((drive_output+heading_output), (drive_output-heading_output));
    logD("STEP PURE", "X: " + FTS(chassis.get_X_position()) + ", Y:" + FTS(chassis.get_Y_position()) + ", Heading:" + FTS(chassis.get_absolute_heading()) + ", TX:" + FTS(X_position) + ", TY:" + FTS(Y_position) + ", driveError:" + FTS(drive_error) + ", headingError:" + FTS(heading_error) + ", driveOutput:" + FTS(drive_output) + ", headingOutput:" + FTS(heading_output));
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  logD("STOP PURE", "Stopping drive to point, error:" + FTS(drive_error));
}


void Drive::drive_to_pointChain(float X_position, float Y_position, float lDist){
  drive_to_pointChain(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti, lDist);
}



void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    // Again, using atan2(x,y) puts 0 degrees on the positive Y axis.
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  DriveL.stop(hold);
  DriveR.stop(hold);
}



void Drive::holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = angle;
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(!(drivePID.is_settled() && turnPID.is_settled() ) ){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float turn_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    float heading_error = atan2(Y_position-get_Y_position(), X_position-get_X_position());

    DriveLF.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) + turn_output, volt);
    DriveLB.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) + turn_output, volt);
    DriveRB.spin(fwd, drive_output*cos(to_rad(get_absolute_heading()) + heading_error - M_PI/4) - turn_output, volt);
    DriveRF.spin(fwd, drive_output*cos(-to_rad(get_absolute_heading()) - heading_error + 3*M_PI/4) - turn_output, volt);
    //These scalings on the drive output ensure that the drive always goes toward the desired point.
    task::sleep(10);
  }
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRB.stop(hold);
  DriveRF.stop(hold);
}

// The usercontrol functions use deadband=5 everywhere. This value pretty much gets the job done,
// but it can be changed with no repercussions.

void Drive::control_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

void Drive::control_holonomic(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  float strafe = deadband(controller(primary).Axis4.value(), 5);
  DriveLF.spin(fwd, to_volt(throttle+turn+strafe), volt);
  DriveRF.spin(fwd, to_volt(throttle-turn-strafe), volt);
  DriveLB.spin(fwd, to_volt(throttle+turn-strafe), volt);
  DriveRB.spin(fwd, to_volt(throttle-turn+strafe), volt);
}

void Drive::control_tank(){
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
}

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}