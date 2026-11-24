#include "Libraries/stringFunctions.h"
#include "botSetup/vex.h"
#include "Libraries/controllerUI.h"

#include <sstream>
#include <random>
#include <iostream>
#include <fstream>
 
std::string floatToString(float number){
  std::ostringstream fixed;
  std::string textNum; 

  fixed << number; 
  textNum = fixed.str();

  return textNum.c_str();
}

std::string FTS(float number){//Float to string
  return floatToString(number);
}

std::string BTS(bool thing){//Bool to string
  return (thing ? "true":"false");
}


float STF(std::string str){//String to float
  return atof(str.c_str());
}
int STI(std::string str){//String to float
  return atoi(str.c_str());
}

//ALso gonna include this function because I have no clue wherelse to put it
float getRandomFloat(float min, float max) {
  //use bult in clock
    std::mt19937 gen(Brain.Timer.systemHighResolution()) ;
    std::uniform_real_distribution<float> dis(min, max);
    return dis(gen);
}

std::string readFile(std::string fileName){
  return "NULL";
  //Get size in bytes of the file
  int len = Brain.SDcard.size(fileName.c_str());
  
  //Make a unsigned character array for loading data to
  unsigned char u_array[len];

  //Load data into the array
  Brain.SDcard.loadfile(fileName.c_str(), u_array, len);

  //Cast from unsigned char array to regular char array to string
  std::string str (reinterpret_cast<const char *> (u_array), len);
  return str;
}

int writeFile(std::string fileName, std::string data){
  return 0;
  return Brain.SDcard.savefile(fileName.c_str(), (uint8_t *)data.c_str(), data.length());
}

int appendFile(std::string fileName, std::string data){
  return 0;
  return Brain.SDcard.appendfile(fileName.c_str(), (uint8_t *)data.c_str(), data.length());
}