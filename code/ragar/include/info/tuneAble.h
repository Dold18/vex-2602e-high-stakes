#ifndef tuneAble_H
#define tuneAble_H

float runCurve(float value);

extern float turnAccuracy;
  //Turning required accuricy in degrees

extern float driveAccuracy;
  //Drive accuricy in cm

extern float turnVelocityAuton;
  //percent

extern float driveVelocityAuton;
  //percent

extern float driveTurnVelocityAuton;
  //pct

extern float zeroAccuracy;
  //pct

extern float zeroAccuracyDrive;
  //pct

extern double rotationToTravel;
  //multiply rotation by this value
  //gear ratio is 5:3
  //Circumferece of wheel is 25.935cm
  //one rotation is 43.255cm

extern float timeoutTurn;
  //seconds

extern float timeoutDrive;
  //seconds

extern float smallDistValue;
  //fake cm


extern float autonRefreshSpeed;
//refresh speed in ms

#endif