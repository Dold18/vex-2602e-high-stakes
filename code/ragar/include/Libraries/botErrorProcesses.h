#ifndef botErrorProcesses_H
#define botErrorProcesses_H

#include <string>

std::string getDisconnectErrors();
std::string getTempMessage();
bool fixDriveMotors(float LeftMotors, float RightMotors);
std::string getHeatMap();
bool intakeOverHeat();

std::string getDiagnostic1();
std::string getDiagnostic2();
std::string getDiagnostic3();

std::string getHeat1();
std::string getHeat2();
std::string getHeat3();
#endif