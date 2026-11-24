#ifndef motorFuncs_H
#define motorFuncs_H

void setMotorVelocities(float speed);

void setup();

void spinToPos(float pos);

void default_constants();

void odom_constants();

void driveToLine(bool forwards, int timeout);

int intakeRing();

int antiJamIntake();

extern bool cancelIntake;

#endif