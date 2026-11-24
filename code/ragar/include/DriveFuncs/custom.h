#ifndef custom_H
#define custom_H
//This defines turns with the custom PID values determined from the PID

#include <string>

int plainTurnProfile(float difference);
int mobileGoal0Profile(float difference);

float turnToHeadingC(float tHeading);
float turnToHeadingC(float tHeading, std::string acc);

void driveDistance(float distance);

void setDriveParams(float inches);
float getAnglePoint(float X_position, float Y_position);

#endif