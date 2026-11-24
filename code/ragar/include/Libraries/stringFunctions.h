#ifndef stringfunc_H
#define stringfunc_H

#include <string>

std::string floatToString(float number);

std::string FTS(float number);

std::string BTS(bool thing);

float STF(std::string str);

int STI(std::string str);

float getRandomFloat(float min, float max);

std::string readFile(std::string fileName);

int writeFile(std::string fileName, std::string data);

int appendFile(std::string fileName, std::string data);

#endif 