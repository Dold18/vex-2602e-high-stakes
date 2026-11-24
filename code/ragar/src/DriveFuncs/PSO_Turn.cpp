#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "info/log.h"
#include "DriveFuncs/PID.h"
 

#include <string>
#include <array> 
#include <random>


bool reversec = false;


int timeTurnTest(float angle){

  chassis.set_heading(0);

  int startTime = Brain.timer(msec);
  if(PSOTy == "TURN"){
    chassis.turn_to_angle(angle);
  }
  else if(PSOTy == "DRIVE"){

  float distance = angle;
  if(reversec == true){
    distance = -distance;
    reversec = false;
  }
  else if(reversec == false){
    reversec = true;
  }
  //chassis.drive_distance(distance);
  donaldDuck(distance);

  }
  else if(PSOTy == "SWING"){
    chassis.left_swing_to_angle(angle);
  }

  return Brain.timer(msec)-startTime;
}

float fullTurnTest(float KP, float KI, float KD, float StartingI, float angle){
  if(hasRunSetup==false){
    hasRunSetup = true;
    vexcodeInit();
    default_constants();
  }
  //StartingI = starting integral value
  //Set max voltage low because otherwise we overheat
  //chassis.set_heading(0);
  //chassis.set_turn_constants(12, .4, .03, 3, 15);


  if(PSOTy == "TURN"){
    //FOR TURNS
    chassis.set_turn_exit_conditions(1, 300, 3000);
    chassis.set_turn_constants(12, KP, KI, KD, StartingI);
  }
  else if(PSOTy == "DRIVE"){
    //FOR DRIVES
    chassis.set_drive_exit_conditions(1, 300, 3000);
    chassis.set_drive_constants(10, KP, KI, KD, StartingI);
  }
  else if(PSOTy == "SWING"){
    //FOR SWEEPS
    chassis.set_swing_exit_conditions(1, 450, 3000);
    chassis.set_swing_constants(12, KP, KI, KD, StartingI);
  }

  float sum = 0;
  //Remove some angles to go quicker, also do PID for each if possible
  //sum += timeTurnTest(45);
  //sum += timeTurnTest(90);
  //sum += timeTurnTest(135);
  //sum += timeTurnTest(180);

  wait(0.2,seconds);//Allow for bot to stop
  sum += timeTurnTest(angle);

  return sum;
}



std::string getPSOPopulation(const int num_particles, int angle){
  clampPistons.set(true);
  float population[num_particles*4];
  float velocities[num_particles*4];

  //Best vlaues and best fitness scores for each member
  float pBest_params[num_particles*4];
  float pBest_values[num_particles];

  //Global best vlaues for all members, gBest_value needs to be very high
  float gBest_params[4];
  //OG
  // gBest_params[0] = float(-0.216);
  // gBest_params[1] = float(.015);
  // gBest_params[2] = float(0.08);
  // gBest_params[3] = float(0.69);
  //90 degree turn USE FOR REAL BOT


  if(PSOTy == "SWING"){
  //FOR SWEEP
  gBest_params[0] = float(.3);
  gBest_params[1] = float(.001);
  gBest_params[2] = float(2);
  gBest_params[3] = float(15);
  }

  if(PSOTy == "DRIVE"){
    gBest_params[0] = float(1.5);
  gBest_params[1] = float(.0);
  gBest_params[2] = float(10);
  gBest_params[3] = float(0);
  }

  if(PSOTy == "TURN"){
  //FOR TURNS
  gBest_params[0] = float(.4516);
  gBest_params[1] = float(.0046);
  gBest_params[2] = float(3.0105);
  gBest_params[3] = float(15.014);
  }
  //This also has to be low
  //float gBest_value = 3800;
  float gBest_value = 2000;//for sing,re turns

  //float scaler = 0.25; USE this for real
  float scaler = 0.5;//1.25;//0.25;

  for(int i = 0; i<(4*num_particles); i+=4){
    // population[i] = getRandomFloat(-scaler, scaler);
    // population[i+1] = getRandomFloat(-scaler, scaler);
    // population[i+2] = getRandomFloat(-scaler, scaler);
    // population[i+3] = getRandomFloat(-2.5 * scaler, 2.5 * scaler);

    population[i] = gBest_params[0] + getRandomFloat(-scaler/3, scaler/3);
    population[i+1] = gBest_params[1] + getRandomFloat(-scaler/7, scaler/7);
    population[i+2] = gBest_params[2] + getRandomFloat(-scaler, scaler);
    population[i+3] = gBest_params[3] + getRandomFloat(-7.5 * scaler, 7.5 * scaler);

    //   population[i] = gBest_params[0] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+1] = gBest_params[1] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+2] = gBest_params[2] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+3] = gBest_params[3] * getRandomFloat( 1 + (-2.5 * scaler), 1 + (2.5 * scaler));
  

    //Set up pbest to try to reduce beginning but keep score higher because we want variation
    pBest_values[i] = 8000;

    pBest_params[i] = population[i];
    pBest_params[i+1] = population[i+1];
    pBest_params[i+2] = population[i+2];
    pBest_params[i+3] = population[i+3];
  }

  std::string populationString = FTS(angle) + "," + FTS(gBest_value) + "," + FTS(gBest_params[0]) + "," + FTS(gBest_params[1]) + "," + FTS(gBest_params[2]) + "," + FTS(gBest_params[3]);

  for(int i = 0; i<(num_particles*4); i+=4){
    std::string pBest = FTS(pBest_values[i/4]) + "," + FTS(pBest_params[0]) + "," + FTS(pBest_params[1]) + "," + FTS(pBest_params[2]) + "," + FTS(pBest_params[3]);
    std::string params = FTS(population[0 + i]) + "," + FTS(population[1 + i]) + "," + FTS(population[2 + i]) + "," + FTS(population[3 + i]);
    std::string velo = FTS(velocities[0 + i]) + "," + FTS(velocities[1 + i]) + "," + FTS(velocities[2 + i]) + "," + FTS(velocities[3 + i]);
    populationString += "," + pBest + "," + params + "," + velo;
  }
  return populationString;
}


void PSOLoop(int num_interations, int num_particles, std::string popData, float angle, float max_velocity, float c1, float c2, std::string SAVEFILE){
  std::string lastText2 = "";
  chassis.set_heading(0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_turn_exit_conditions(1, 300, 3000);

  float population[num_particles*4];
  float velocities[num_particles*4];
  float pBest_params[num_particles*4];
  float pBest_values[num_particles];
  float gBest_params[4];
  float gBest_value;


  std::string delimiter = ",";

  int pos = popData.find(delimiter);
  int popAngle = STF(popData.substr(0, pos));
  popData = popData.substr(pos + delimiter.length());
 
  if(popAngle != angle){
	  displayInfo(Controller1, "ERROR", "ANGLE!=", "PSOLoop()");
    logD("ERROR PSO Turn", "Population angle != turn angle entered", true);
  }
 
  pos = popData.find(delimiter);
  gBest_value = STF(popData.substr(0, pos));
  popData = popData.substr(pos + delimiter.length());

  for(int i = 0; i<4; i++){
	pos = popData.find(delimiter);
	gBest_params[i] = STF(popData.substr(0, pos));
	popData = popData.substr(pos + delimiter.length());
  }
 
 

  for(int p = 0; p<num_particles*4; p+=4){
 	 
	pos = popData.find(delimiter);
	pBest_values[p/4] = STF(popData.substr(0, pos));
	popData = popData.substr(pos + delimiter.length());
	
    
  	for(int i = 0; i<4; i++){
    	pos = popData.find(delimiter);
    	pBest_params[p + i] = STF(popData.substr(0, pos));
    	popData = popData.substr(pos + delimiter.length());
  	}
 	 
  	for(int i = 0; i<4; i++){
    	pos = popData.find(delimiter);
    	population[p + i] = STF(popData.substr(0, pos));
    	popData = popData.substr(pos + delimiter.length());
  	}
 	 
  	for(int i = 0; i<4; i++){
    	pos = popData.find(delimiter);
    	velocities[p + i] = STF(popData.substr(0, pos));
    	popData = popData.substr(pos + delimiter.length());
  	}
 
    
  }
  

  for(int u = 0; u<num_interations; u+=1){   
    logD("PSO TURN GENERATION", "Interation:" + FTS(u) + ", GBEST Fitness:" + FTS(gBest_value) + ", GBEST KP:" + FTS(gBest_params[0]) + ", GBEST KI:" + FTS(gBest_params[1]) + ", GBEST KD:" + FTS(gBest_params[2]) + ", StartI:" + FTS(gBest_params[3]), true);
    if(SAVEFILE != "NONE"){
      std::string populationString = FTS(angle) + "," + FTS(gBest_value) + "," + FTS(gBest_params[0]) + "," + FTS(gBest_params[1]) + "," + FTS(gBest_params[2]) + "," + FTS(gBest_params[3]);

      for(int i = 0; i<(num_particles*4); i+=4){
        std::string pBest = FTS(pBest_values[i/4]) + "," + FTS(pBest_params[0]) + "," + FTS(pBest_params[1]) + "," + FTS(pBest_params[2]) + "," + FTS(pBest_params[3]);
        std::string params = FTS(population[0 + i]) + "," + FTS(population[1 + i]) + "," + FTS(population[2 + i]) + "," + FTS(population[3 + i]);
        std::string velo = FTS(velocities[0 + i]) + "," + FTS(velocities[1 + i]) + "," + FTS(velocities[2 + i]) + "," + FTS(velocities[3 + i]);
        populationString += "," + pBest + "," + params + "," + velo;
      }
      writeFile(SAVEFILE, populationString);
    }

      for(int i = 0; i<(num_particles*4); i+=4){
        while(true){
          if((RightMiddleMotor.temperature(celsius)<55 && LeftMiddleMotor.temperature(celsius)<55) && (!Controller1.ButtonX.pressing())){
            
            population[i+1] = fabs(population[i+1]);
            float fitness = fullTurnTest(population[i], population[i+1], population[i+2], population[i+3], angle);
            std::string runInfo = ", Fitness:" + FTS(fitness) + ", KP:" + FTS(population[i]) + ", KI:" + FTS(population[i+1]) + ", KD:" + FTS(population[i+2]) + ", StartI:" + FTS(population[i+3]);
            std::string pBestInfo = ", PFitness:" + FTS(pBest_values[i/4]) + ", PKP:" + FTS(pBest_params[i]) + ", PKI:" + FTS(pBest_params[i+1]) + ", PKD:" + FTS(pBest_params[i+2]) + ", PStartI:" + FTS(pBest_params[i+3]);
            std::string gBestInfo = ", GFitness:" + FTS(gBest_value) + ", GKP:" + FTS(gBest_params[0]) + ", GKI:" + FTS(gBest_params[1]) + ", GKD:" + FTS(gBest_params[2]) + ", GStartI:" + FTS(gBest_params[3]);
            


            
        

            if (fitness < pBest_values[i]){
                  pBest_params[i] = population[i];
                  pBest_params[i+1] = population[i+1];
                  pBest_params[i+2] = population[i+2];
                  pBest_params[i+3] = population[i+3];
                  pBest_values[i/4] = fitness;// this wasn't divided by 4!
            }

          // Update global best (gbest) position and value
          if (fitness < gBest_value) {
            //double check to make sure the values are actually better
            fitness = fullTurnTest(population[i], population[i+1], population[i+2], population[i+3], angle);
            
            if(fitness < gBest_value){
              logD("PSO TURN PARTICLE GBEST", "Interation:" + FTS(u) + ", Particle:" + FTS(i/4) + ", Fitness:" + FTS(fitness) + ", KP:" + FTS(population[i]) + ", KI:" + FTS(population[i+1]) + ", KD:" + FTS(population[i+2]) + ", StartI:" + FTS(population[i+3]), true);
              gBest_params[0] = population[i];
              gBest_params[1] = population[i+1];
              gBest_params[2] = population[i+2];
              gBest_params[3] = population[i+3];
              gBest_value = fitness;
            }
            else{
              logD("PSO TURN PARTICLE", "Interation:" + FTS(u) + ", Particle:" + FTS(i/4) + ", Fitness:" + FTS(fitness) + ", KP:" + FTS(population[i]) + ", KI:" + FTS(population[i+1]) + ", KD:" + FTS(population[i+2]) + ", StartI:" + FTS(population[i+3]), true);
            }
          }

          //END SCORING
          // Update the velocity and position of the particle

          for(int g = 0; g<4; g+=1){
            float scaler = 1;
            if(g==0){
              scaler = 0.5;
            }
            if(g==1){
              scaler = 0.03;
            }
            if(g==2){
              scaler = 1;
            }
            if(g==0){
              scaler = 2;
            }
            float new_velocity = (velocities[i + g] + getRandomFloat(-0.005,0.005)+ c1 * getRandomFloat(0,1) * (pBest_params[i + g] - population[i+g]) + c2 * getRandomFloat(0,1) * (gBest_params[g] - population[i+g]));
            new_velocity = scaler * new_velocity;
            //limit velocity of the particle
            if(new_velocity>(max_velocity*scaler)){
              new_velocity = max_velocity * scaler;
            }
            if(new_velocity<-(max_velocity*scaler)){
              new_velocity = -max_velocity * scaler;
            }

            population[i + g] += (new_velocity/8);
            velocities[i + g] = new_velocity;


            
          }

          std::string velocityInfo = ", vKP:" + FTS(velocities[i]) + ", vKI:" + FTS(velocities[i+1]) + ", vKD:" + FTS(velocities[i+2]) + ", vStartI:" + FTS(velocities[i+3]);
            logD("PSO TURN PARTICLE", "Interation:" + FTS(u) + ", Particle:" + FTS(i/4) +  runInfo + pBestInfo + gBestInfo + velocityInfo, true);

      //Give a quick update on how its going
      if(Controller1.ButtonY.pressing()){
        displayInfo(Controller1, floatToString(gBest_params[0]) + " " + floatToString(gBest_params[1]), floatToString(gBest_params[2]) + " " + floatToString(gBest_params[3]), floatToString(gBest_value));
      }
      else{
        displayInfo(Controller1, floatToString(population[i]) + " " + floatToString(population[i+1]), floatToString(population[i+2]) + " " + floatToString(population[i+3]), floatToString(fitness) + " " + floatToString(i/4));
      }
      break;
      }
      else{
        
        LeftMotors.setVelocity(0, pct);
        RightMotors.setVelocity(0, pct);

        LeftMotors.setStopping(coast);
        RightMotors.setStopping(coast);
        
        std::string firstLine;
        std::string secondLine;
        std::string thirdLine;
          firstLine = getDiagnostic1();
          secondLine = getDiagnostic2();
          thirdLine = getDisconnectErrors();
          if (thirdLine == "") {
            thirdLine = getDiagnostic3();
          }

        std::string newText = firstLine + secondLine + thirdLine;
        if (lastText2 != newText) {
          displayInfo(Controller1, firstLine, secondLine, thirdLine);
        }
        lastText2 = newText;

        logD("CRITICAL OVERHEAT PSO TURN", firstLine + ":" + secondLine + ":" + thirdLine, true);
    
        wait(15,seconds);
      }


      }
    }
  }

  //Display output
  displayInfo(Controller1, floatToString(gBest_params[0]) + " " + floatToString(gBest_params[1]), floatToString(gBest_params[2]) + " " + floatToString(gBest_params[3]), floatToString(gBest_value));

  logD("STOP PSO TURN", floatToString(gBest_params[0]) + " " + floatToString(gBest_params[1]) + " " + floatToString(gBest_params[2]) + " " + floatToString(gBest_params[3]) + " " + floatToString(gBest_value), true);

}




//This includes the actual PSO loops and tests for turning
//std::string lastText2 = "";
void TurningPSO(const int num_particles, int num_interations, float max_velocity, float c1, float c2, float angle){
  logD("START PSO TURN", "numParticles:" + FTS(num_particles) + ", numInterations:" + FTS(num_interations) + ", maxVelocity:" + FTS(max_velocity) + ", c1:" + FTS(c1) + ", c2:" + FTS(c2) + ", angle:" + FTS(angle), true);
  
  

  std::string pop = getPSOPopulation(num_particles, angle);
  PSOLoop(num_interations, num_particles, pop, angle, max_velocity, c1, c2, "NONE");
}
