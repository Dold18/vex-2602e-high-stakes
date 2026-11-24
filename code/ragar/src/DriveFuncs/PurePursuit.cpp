#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "info/points.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "info/log.h"

#include <string>
#include <array>
#include <random> 
    
   
   
double cxPos = 0, cyPos = 0, frontTracker = 0;
bool dReverse = false;
  
// void odom_Track(){
//   double distMoved = (LeftMotors.rotation(vex::degrees)+RightMotors.rotation(vex::degrees))/2;//avg and then by 360
//   distMoved *= (0.6 * 10.21018)/360;//gear ratio times circumfrece
//   cxPos += sin(to_rad(chassis.get_absolute_heading()))*(distMoved-frontTracker);
//   cyPos += cos(to_rad(chassis.get_absolute_heading()))*(distMoved-frontTracker);
//   frontTracker = distMoved;
// }

// float getDistance(float px, float py){
// double x = pow((cxPos - px), 2);
// double y = pow((cyPos - py), 2);
// return sqrt(x+y);
// }

// void tdrive(){
//   if(!hasRunSetup){
//     hasRunSetup = true;
//     vexcodeInit();
//     default_constants();
//   }
//   LeftMotors.resetRotation();
//   RightMotors.resetRotation();
//   cxPos=cyPos=0;

//   setMotorVelocities(0);

//   while(true){
//     LeftMotors.setVelocity(Controller1.Axis3.value(), pct);
//     RightMotors.setVelocity(Controller1.Axis2.value(), pct);
//     displayInfo(Controller1, floatToString(cxPos),floatToString(cyPos),floatToString(chassis.get_absolute_heading()));
//     odom_Track();
//     wait(10,msec);
//   }
// }

float peerPursuit(float Timeout, int currentPoint, int endPoint, int pointList){

  // LeftMotors.resetRotation();
  // RightMotors.resetRotation();
  logD("START PURE", "timeout:" + FTS(Timeout) + ", currentPoint:" + FTS(currentPoint) + ", endPoint:" + FTS(endPoint) + ", pointList:" + FTS(pointList));

  float startTime = Brain.timer(msec);

  if(endPoint==-1){
    if(pointList==0){
      endPoint = length;
    }
    if(pointList==1){
      endPoint = length1;
    }
     if(pointList==2){
      endPoint = length2;
    }
     if(pointList==3){
      endPoint = length3;
    }
    if(pointList==4){
      endPoint = length4;
    }
    if(pointList==5){
      endPoint = length5;
    }
    
  }
  
  float proX; 
  float proY;
  while(currentPoint<endPoint && (Brain.timer(msec)-startTime)<Timeout){
    //It previously would try to run with the length value point and would overflow
    // look ahead point calculation
    if(pointList==0){
      proX = pointx[currentPoint];
      proY = pointy[currentPoint];
    }
    if(pointList==1){
      proX = pointx1[currentPoint];
      proY = pointy1[currentPoint];
    }
    if(pointList==2){
      proX = pointx2[currentPoint];
      proY = pointy2[currentPoint];
    }
    if(pointList==3){
      proX = pointx3[currentPoint];
      proY = pointy3[currentPoint];
    }
    if(pointList==4){
      proX = pointx4[currentPoint];
      proY = pointy4[currentPoint];
    }
    if(pointList==5){
      proX = pointx5[currentPoint];
      proY = pointy5[currentPoint];
    }

    logD("LOG PURE POINT", "STEP 0");
      logD("LOG PURE POINT", "PointX: " + FTS(proX) + ", PointY: " + FTS(proY) + ", PointIndex: " + FTS(currentPoint), true );
      chassis.drive_to_point(proX, proY);
      logD("LOG PURE POINT", "STEP 1");
      currentPoint++;
      logD("LOG PURE POINT", "STEP 2");

    displayInfo(Controller1, floatToString(currentPoint));
    logD("LOG PURE POINT", "STEP 3");
     
  }  
  displayInfo(Controller1,"C4"); 
  
  logD("STOP PURE", "Ending pure loop", true);

  return Brain.timer(msec)-startTime;
}

/* 
  while(true){
    // follower logic
    dist = getDistance(pointx[currentPoint], pointy[currentPoint]);
    // advance current point based on distance
    while(dist<LDist){
      currentPoint++;
      dist = getDistance(pointx[currentPoint], pointy[currentPoint]);
    };
    // look ahead point calculation

    float proX = (pointx[currentPoint] - cxPos);
    float proY = (pointy[currentPoint] - cyPos);


    float targetX, targetY;

    if(currentPoint < length-1){
      targetX = pointx[currentPoint+1]; 
      targetY = pointy[currentPoint+1];
    }
    else {
      targetX = pointx[currentPoint];
      targetY = pointy[currentPoint];
    }

    // calculate look ahead point
    float lookAheadX = targetX + LDist * (targetX-cxPos)/sqrt(pow(targetX-cxPos,2)+pow(targetY-cyPos,2)); 
    float lookAheadY = targetY + LDist * (targetY-cyPos)/sqrt(pow(targetX-cxPos,2)+pow(targetY-cyPos,2));

    // calculate steering angle to look ahead point
    float targetAngle = reduce_0_to_360(-to_deg(atan2(-(lookAheadX - cxPos), lookAheadY - cyPos)));

    // calculate steering command
    float steering = reduce_negative_180_to_180(targetAngle - chassis.get_absolute_heading());
    

    // drive motors
    

    float forwardVel = (6*(dist-4)) * cos(to_rad(1*steering));//100 - (pow(0.8 * fabs(deltaAngle), 1.7) * (dist/15));
    if(forwardVel<0){
      forwardVel = 0;
    }

    forwardVel = to_volt(forwardVel);

    steering = clamp(steering/6, -6, 6);

    displayInfo(Controller1,  floatToString(proX)+", "+floatToString(proY), floatToString(steering), floatToString(currentPoint)+", "+floatToString(dist));

    if(Controller1.ButtonA.pressing()){
      setMotorVelocities(0);
      // LeftMotors.spin(forward, forwardVel + steering/2, volt);
      // RightMotors.spin(forward, forwardVel - steering/2, volt);
      LeftMotors.setVelocity((forwardVel + steering/2)*(100/12), pct);
      RightMotors.setVelocity((forwardVel - steering/2)*(100/12), pct);
    }else{
      LeftMotors.stop(coast);
      RightMotors.stop(coast);
    }

    if(length<=currentPoint || Brain.timer(msec)-startTime>Timeout){
      break;
    }

    odom_Track();
    wait(5,msec);

  }

  setMotorVelocities(0);
  displayInfo(Controller1,"C4");
  return Brain.timer(msec) - startTime;

}*/