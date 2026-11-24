#include "DriveFuncs/motorFuncs.h"
#include "botSetup/vex.h"
#include "botSetup/frontEye.h"
#include "info/settings.h"
#include "info/log.h"
#include "Libraries/ControllerUI.h"
#include "Libraries/stringFunctions.h"



 

bool tracking = false;
float xPos = -1;
float yPos = -1;

const int camWidth = 316;
const int camHeight = 212;
const int HFOV = 61;
const int VFOV = 41;

optical intakeDetector(PORT21);






void driveToRing(std::string color, float zoneX, float zoneY){
  logD("START DRIVETORING", "Color:" + color + ", zoneX:" + FTS(zoneX) + ", zoneY:" + FTS(zoneY));
  frontEye.takeSnapshot(1);

  intakeDetector.integrationTime(30);
  intakeDetector.objectDetectThreshold(100);

  if(tracking==false){
    xPos = frontEye.largestObject.centerX;
    yPos = frontEye.largestObject.centerY;
  }

  bool intakeRing = false;

  int count = 0;

  float kp = 1.5;
  float ki = 0;
  float kd = 10;
  float startI = 0;
  float maxVolt = 10;
  float headingMaxVolt = chassis.heading_max_voltage;

  PID headingPID(0, chassis.heading_kp, chassis.heading_ki, chassis.heading_kd, chassis.heading_starti);
  PID drivePID(10, kp, ki, kd, startI);

  while(intakeRing == false){
    frontEye.takeSnapshot(1);


    float minDist = 100000;
    int minIndex = -1;

    for(int i = 0; i<frontEye.objectCount; i++){
      float dist = sqrt(pow((frontEye.objects[i].centerX-xPos), 2) + pow((frontEye.objects[i].centerY-yPos), 2));
      if(dist<minDist){
        minDist = dist;
        minIndex = i;
      }
    }
    vision::object target = frontEye.objects[minIndex];

    xPos = target.centerX;
    yPos = target.centerY;

    float normalized_x = (xPos - (camWidth / 2)) / (camWidth / 2);
    float normalized_y = (yPos - (camHeight / 2)) / (camHeight / 2);

    float horizontal_angle = normalized_x * (HFOV / 2);
    float vertical_angle = normalized_y * (VFOV / 2);//THIS IS BAD NOT ANYMORE







    float drive_error = 4 + 0.5 * ((VFOV/2) - vertical_angle);
    // The drive error is just equal to the distance between the current and desired points.
    float heading_error = horizontal_angle;
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
    
    if (drive_error<chassis.drive_settle_error) { heading_output = 0; }
    // This if statement prevents the heading correction from acting up after the robot gets close
    // to being settled.

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*maxVolt, fabs(heading_scale_factor)*maxVolt);
    heading_output = clamp(heading_output, -headingMaxVolt, headingMaxVolt);

    chassis.drive_with_voltage((drive_output+heading_output), (drive_output-heading_output));

    logD("STEP DRIVETORING", "Time:" + FTS(count) + ", hAngle: " + FTS(horizontal_angle) + ", vAngle: " + FTS(vertical_angle) + ", xPos: " + FTS(xPos) + ", yPos: " + FTS(yPos) + ", X: " + FTS(chassis.get_X_position()) + ", Y:" + FTS(chassis.get_Y_position()) + ", Heading:" + FTS(chassis.get_absolute_heading()) + ", driveError:" + FTS(drive_error) + ", headingError:" + FTS(heading_error) + ", driveOutput:" + FTS(drive_output) + ", headingOutput:" + FTS(heading_output));
    wait(10,msec);
    count += 10;

    if(intakeDetector.isNearObject())
      intakeRing = true;
  }

  LeftMotors.stop(hold);
  RightMotors.stop(hold);

  logD("STOP DRIVETORING", "Time:" + FTS(count));
}