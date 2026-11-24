#include "DriveFuncs/motorFuncs.h"
#include "botSetup/vex.h"
#include "info/settings.h"
#include "info/log.h"
#include "Libraries/ControllerUI.h"
#include "Libraries/stringFunctions.h"




float speedLB = 1; 

PID lbPID(0, 0.4, 0.002, 1.1, 6, false);

void spinToPos(float pos){
  //float ang = ladyBrownTrack.position(degrees);
  float ang = -bMech.position(degrees)/3;
    // Reducing the angle to a value between -180 and 180 degrees ensures that the robot always takes the 
    // shorter path when making a turn.
    float output = lbPID.compute(ang-pos);
    float mx = 12;
    output = clamp(output, -mx, mx);
  
  bMech.spin(forward, output*speedLB, volt);
  if(fabs(ang-pos)<2){
    bMech.stop(hold);
  }
}

float tPosLB = 0;

bool spinTask = false;

int floorCounter = 0;
bool needReset = false;

int spinToPosTask(){
  if(spinTask == true) return 1;
  
  spinTask = true;
  while(spinTask==true){
    bMech.setStopping(hold);


    if(tPosLB < 2 && needReset == true){
      

      float ang = -bMech.position(degrees)/3;
      // if(fabs(ang+tPosLB)<30){
      //   tPosLB-=50;
      // }
      tPosLB = -3000;
      if(fabs(bMech.velocity(rpm))<1.5){
        floorCounter++;
      }
      else{
        floorCounter=0;
      }
      //displayInfo(Controller1, FTS(floorCounter), FTS(bMech.velocity(rpm)));
      if(floorCounter>15 && fabs(bMech.velocity(rpm))<1.5){// && ang>-10){
        //bMech.stop(coast);
        bMech.setPosition(28, degrees);
        needReset = false;
        tPosLB = 1;
      }
    }
    else if(tPosLB<2){
      floorCounter = 0;
    }
    else{
      needReset = true;
      floorCounter = 0;
    }
    
    spinToPos(tPosLB);
    
    wait(10,msec);
  }
  spinTask=false;
  return 0;
}