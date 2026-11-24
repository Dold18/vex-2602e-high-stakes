#include "botSetup/vex.h"

#include <cmath>

#include <string>

#include "info/tuneAble.h"

#include "info/settings.h" 

#include "Libraries/controllerUI.h"

#include "Libraries/stringFunctions.h"

#include "botSetup/robot-config.h"

#include "info/log.h"

#include "JAR-Template/util.h"

#include "Sensing/distance.h"



//doesnt work 
float distanceGetAngle(){
  

  float d1 = 1;//leftSonar.distance(inches);
  float d2 = 1;//rightSonar.distance(inches);

  double distBetweenSensors = 10;
  double angle = atan2(d2-d1, distBetweenSensors);

  return reduce_0_to_360( to_deg(angle) );

}
//doen wrok
float distanceGetY(float angle){
  float d1 = 1;//leftSonar.distance(inches);
  float d2 = 1;//rightSonar.distance(inches);


  //double a1 = to_rad(20);
  float yPos = 0;

  if(angle<180){
    yPos = cos(to_rad(angle)) * d1;
  }
  else{
    yPos = cos(to_rad(angle)) * d2;
  }

  return fabs(yPos);
}







float getDistX(bool isRight, float angle){
  float distPos = -1;
  if(isRight == false){
    angle += 180;
  }

  if(reduce_0_to_360(angle)>330 || reduce_0_to_360(angle)<30){
      distPos = cos(to_rad(angle)) * (rightDist.objectDistance(inches) - 1.2);
    }
    else if(reduce_0_to_360(angle)<300 && reduce_0_to_360(angle)>240){
      distPos = cos(to_rad(angle-270)) * (backDist.objectDistance(inches) - 3.8);
    }
    else if(reduce_0_to_360(angle)<210 && reduce_0_to_360(angle)>150){
      distPos = cos(to_rad(angle-180)) * (leftDist.objectDistance(inches) - 1.1);
    }
    else if(reduce_0_to_360(angle)<120 && reduce_0_to_360(angle)>60){
      distPos = cos(to_rad(angle-90)) * (frontDist.objectDistance(inches) - 1.0);
    }

    //distPos = cos(to_rad(angle)) * frontDist.objectDistance(inches);
    
  //logD("LOG", "IsRight:" + BTS(isRight) + ", distPos:" + FTS(distPos));
  if(distPos>150){
    logD("ERROR DISTANCE", "Distance found no range");
    distPos = -1;
  }
  if(distPos < 0.1){
    distPos = -1;
  }


  return distPos;
}


float getDistY(bool isRight, float angle){
  ///displayInfo(Controller1, FTS(cos(to_rad(angle)) * frontDist.objectDistance(inches)));
  return getDistX(true, angle-90);
}


int getPosDist(bool isRight, float angle, int waitTime, float distOff){
  double sumX = 0;
  double sumY = 0;
  int samplesX = 0;
  int samplesY = 0;
  for(int i = 0; i<=waitTime; i+=10){
    float xSample = getDistX(isRight, angle);
    float ySample = getDistY(isRight, angle);
    float distX = 64 - xSample;
    float distY =  64 - ySample;
    if(xSample>0){
      if(isRight==false){
        distX = -distX;
      }

      sumX += distX;
      samplesX++;
    }
    if(ySample>0){
      distY = -distY;
      sumY += distY;
      samplesY++;
    }
    
    

    
    
    //chassis.set_coordinates(10, 25, 15);
      if(i+10<=waitTime){
        wait(10, msec);
      }
      else{
        //displayInfo(Controller1, "MARK1");
      }
    }


    float avgX = samplesX==0 ? -100000: sumX/samplesX;
    float avgY = samplesY==0 ? -100000: sumY/samplesY;

    float currentError = sqrt(pow(avgX-chassis.get_X_position(), 2) + pow(avgY-chassis.get_Y_position(), 2));
    if(currentError<distOff || (fabs(avgX-chassis.get_X_position())<8 && fabs(avgY-chassis.get_Y_position())<8)){
      chassis.set_coordinates(avgX, avgY, chassis.get_absolute_heading());
      //logD("ERROR DISTANCE", "Distance too far off");
      //displayInfo(Controller1, FTS(avgX), FTS(avgY), FTS(currentError));
    }
    else{
      if(fabs(avgX-chassis.get_X_position())<distOff/2){
        chassis.set_coordinates(avgX, chassis.get_Y_position(), chassis.get_absolute_heading());
      }
      if(fabs(avgY-chassis.get_Y_position())<distOff/2){
        chassis.set_coordinates(chassis.get_X_position(), avgY, chassis.get_absolute_heading());
      }
    }
    //displayInfo(Controller1, FTS(avgX), FTS(avgY), FTS(currentError));
    //wait(100,msec);
  return 0;
}


int checkForDistTask(float targetAngle, bool isRight, int timeout){
  float mxError = 5;
  int counter = 0;
  while(timeout>counter){
    int testAngle = int( reduce_0_to_360(chassis.get_absolute_heading()) ) % 90;
    //logD("LOG CFDT", "TestAngle:" + FTS(testAngle) + ", heading:" + FTS(chassis.get_absolute_heading()) + ", tg:" + FTS(targetAngle) + ", DLeft:" + FTS(leftDist.objectDistance(inches)) + ", DFront:" + FTS(frontDist.objectDistance(inches)));

    if( (90-mxError)<=(testAngle) || mxError>=(testAngle)){
      //Assume they dont want to measure at 45 degrees BECASUE OYU CANT IDITO
      if(mxError > fabs(reduce_0_to_360(targetAngle - 45) - reduce_0_to_360(chassis.get_absolute_heading() - 45)) ){
        getPosDist(isRight, chassis.get_absolute_heading(), 5, 8);
        return 0; 
      }
    }

    counter+=5;
    wait(5,msec);
  }
  //displayInfo(Controller1, "FAIL");
  return 0;
}


int distTask(bool isRight, int timeout, float mxError){
  int counter = 0;
  while(timeout>counter){
    int testAngle = int( reduce_0_to_360(chassis.get_absolute_heading()) ) % 90;
    
    if( (90-mxError)<=(testAngle) || mxError>=(testAngle)){
      //logD("LOG DT", "TestAngle:" + FTS(testAngle) + ", heading:" + FTS(chassis.get_absolute_heading()) + ", DLeft:" + FTS(leftDist.objectDistance(inches)) + ", DFront:" + FTS(frontDist.objectDistance(inches)));

      //Assume they dont want to measure at 45 degrees BECASUE OYU CANT IDITO
      //if(mxError > fabs(reduce_0_to_360(targetAngle - 45) - reduce_0_to_360(chassis.get_absolute_heading() - 45)) ){
        getPosDist(isRight, chassis.get_absolute_heading(), 5, 8);
      //}
    }

    counter+=5;
    wait(5,msec);
  }
  //displayInfo(Controller1, "FAIL");
  return 0;
}


float getDistance(float x, float y){
  return sqrt(pow(chassis.get_X_position()-x, 2) + pow(chassis.get_Y_position()-y, 2));
}