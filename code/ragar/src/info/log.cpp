#include "info/log.h"
#include "info/settings.h"
#include "botSetup/vex.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/controllerUI.h"
#include <string>
#include <iostream>
#include <sstream>


std::string LOG = "";

std::string logName = "";
 
bool logPIDSteps = true;

//SD card logging of PSO training 
bool PSOLogging = true;
 
int logCounter = 0;

void logD(std::string header, std::string data, bool save){
  if(logEnabled==false){
    return; //log disabled or no sd card, exit program
  }
  return;

  //Add to the log String
  LOG += floatToString(Brain.timer(msec)) + "- " + header + " - " + data + "\n";
  logCounter++; //Increment # of logs

  //If we don't know file name, then find it
  if(logName == ""){
    vex::task nullPtr = task(startLog); //Async task to get log name
  }

  //Save data if save is true or if there has been over 50 logs since last save
  if(save == true || logCounter>50){
    logCounter=0; //Reset counter
    vex::task nullPtr = task(logDTask); //Async, append the data to the file
  }
}

int startLog(){
  //Find log index from master file
  std::string master = "/VEX/LOGS/MASTER.txt";
  std::string testData = readFile(master);
  
  //Convert String to integer
  int currentNumber = STI(testData);

  //Increment the current log file number
  currentNumber++;

  //Write new log index to master file
  std::string newContent = FTS(currentNumber);
  writeFile(master, newContent);

  //Generate log filename and save a log to it
  logName = "LOG" + newContent + ".txt";
  writeFile("/VEX/LOGS/" + logName, LOG);

  //Clear unsaved log variable
  LOG = "";
  return 0;//exit
}


bool loggingAlready = false;

int logDTask(){
  //Don't want to log twice at once
  if(loggingAlready == true) return 0;
  loggingAlready = true;

  //The log may get overwritten otherwise because LOG
  //Would be set after appendfile with takes 11-12msec
  std::string tempLog = LOG;
  
  LOG = "";
  appendFile("/VEX/LOGS/" + logName, tempLog);
  
  //Allow for new logs to be written
  loggingAlready = false;
  return (0);//Exit
}

