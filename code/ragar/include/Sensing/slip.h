#ifndef SLIP_H
#define SLIP_H


void testSlip();
float forceAcceleration(float target, float past);
double linearAccel(double desiredVolts, double currentVolts);
void RUSH(float desiredHeading);

float slew(float target, float current, float maxChange);

#endif