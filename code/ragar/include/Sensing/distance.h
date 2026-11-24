#ifndef DIST_H
#define DIST_H



float distanceGetAngle();

float distanceGetY(float angle);

float getDistX(bool isRight, float angle);
float getDistY(bool isRight, float angle);
int getPosDist(bool isRight, float angle, int waitTime, float distOff);

float getDistance(float x, float y);

int checkForDistTask(float targetAngle, bool isRight, int timeout);

int distTask(bool isRight, int timeout, float mxError);

#endif