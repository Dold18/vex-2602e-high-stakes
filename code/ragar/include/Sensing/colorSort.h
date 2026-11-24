#ifndef colorShort_H
#define colorShort_H

#include <string>

std::string getRingColor();

int runIntake();

extern float intakeVelocity;

extern bool intakeTask;

void setTargetIntake(float target);

extern bool runLower;

extern bool reverseLower;

extern bool runTop;

extern bool stopAtSwitch;

extern bool antiGen;

extern bool runLowerForward;

#endif