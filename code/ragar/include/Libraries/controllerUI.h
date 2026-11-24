#ifndef CONTROLLERUI_H
#define CONTROLLERUI_H

#include "botSetup/vex.h"
#include <string>

//Prints on all three lines using the strings provided
int reprint(controller CONTROLLER, int index, std::string b1="", std::string b2="", std::string b3="");

//Makes a selector that has 2 options, not sure if the third one works
std::string controllerSelector(controller CONTROLLER, std::string b1, std::string b2="", std::string b3="");


std::string controllerSelectorEXT(controller CONTROLLER, const int length, std::string b[length]);


//Makes a selector that has 2 options, not sure if the third one works.
//This has a confirmation screen making sure you selected the correct option
std::string confirmControllerSelector(controller CONTROLLER, std::string b1, std::string b2="", std::string b3="", bool title=false, std::string confText="Confirm?");


// this is the supported function to display information
void displayInfo(controller CONTROLLER, std::string l1="", std::string l2="", std::string l3="");

extern bool displaying; // prevent overwriting

#endif