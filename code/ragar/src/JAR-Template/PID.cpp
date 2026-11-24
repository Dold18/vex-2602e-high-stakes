#include "botSetup/vex.h"
#include "info/settings.h"
#include "Libraries/stringFunctions.h"
#include "info/log.h"

PID::PID(float error, float kp, float ki, float kd, float starti) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd), 
  starti(starti)
{};

PID::PID(float error, float kp, float ki, float kd, float starti, bool logPID) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd), 
  starti(starti),
  logPID(logPID)
{};

PID::PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout) :
  error(error),
  kp(kp),
  ki(ki),
  kd(kd),
  starti(starti),
  settle_error(settle_error),
  settle_time(settle_time),
  timeout(timeout)
{};

//NOTEBOOKE THIS
float PID::compute(float error){
  if (fabs(error) < starti){ // StartI is used to prevent integral windup.
    accumulated_error+=error;
  }
  if ((error>0 && previous_error<0)||(error<0 && previous_error>0)){ 
    accumulated_error = 0; 
  } // This if statement checks if the error has crossed 0, and if it has, it eliminates the integral term.
  
  double kpV = kp*error;
  double kiV = ki*accumulated_error;
  double kdV = kd*(error-previous_error);
  if(fabs(kdV)>50){
    kdV=0;
  }
  output = clamp(kpV + kiV + kdV,-12,12);

  

  

  if(fabs(error)<settle_error){
    //time_spent_settled+=10;
    //New code to check if bot is spinning slow
    if(fabs((error-previous_error))<0.05){
      time_spent_settled+=100;
    }
    else if(fabs((error-previous_error))<0.1){
      time_spent_settled+=60;
    }
    else if(fabs((error-previous_error))<0.2){
      time_spent_settled+=30;
    }
    else{
      time_spent_settled+=10;
    }

  } else {
    time_spent_settled = 0;
  }

  previous_error=error;

  time_spent_running+=10;
  if(logPIDSteps == true && logPID==true){
    logD("STEP PID", "Error:" + FTS(error) + ", kp:" + FTS(kpV) + ", ki:" + FTS(kiV) + ", kd:" + FTS(kdV) + ", TSR:" + FTS(time_spent_running) + ", TSS:" + FTS(time_spent_settled)+ ", OUTPUT:" + FTS(output));
  }

  return output; 
}

bool PID::is_settled(){
  if (time_spent_running>timeout && timeout != 0){
    return(true);
  } // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the 
    // equivalent of setting it to infinity.
  if (time_spent_settled>settle_time){
    return(true);
  }
  return(false);
}