#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"
#include "info/log.h" 

#include <string>
#include <array>
#include <random>
 
//bool reversec = false;

int timeDriveTest(float distance){ 
  if(hasRunSetup==false){
    hasRunSetup = true;
    vexcodeInit();
    default_constants();
  }
  
  // if(reversec == true){
  //   distance = -distance;
  //   reversec = false;
  // }
  // else if(reversec == false){
  //   reversec = true;
  // }

  int startTime = Brain.timer(msec);
  chassis.drive_distance(distance);
  
  int endTime = Brain.timer(msec)-startTime;

  return endTime;
}

float fullDriveTest(float KP, float KI, float KD, float StartingI){
  //StartingI = starting integral value
  chassis.set_drive_constants(10, KP, KI, KD, StartingI);

  //Reset distance
  //setMotorVelocities(-30);
  //wait(1, seconds);

  float sum = 0;
  //We need to account for if the bot drove too far
  //Remove some dists to go quicker, also do PID for each if possible
  sum += timeDriveTest(50);


//This would be for a general PID but those are bad so we only do one at a time
//   sum += timeDriveTest(15 + chassis.get_ForwardTracker_position());
//   //Drove way too far and need to move back a bit

//   if(chassis.get_ForwardTracker_position()>25){
//     //Reset PID to ensure we dont die by trying to use a shit PID
//     chassis.set_drive_constants(10, 1.5, 0, 10, 0);
//     //Move back to where we should be
//     chassis.drive_distance(10 - chassis.get_ForwardTracker_position());
//     chassis.set_drive_constants(12, KP, KI, KD, StartingI);
//   }

//   sum += timeDriveTest(25 + chassis.get_ForwardTracker_position());

// //This is if the bot isn't far enough forwards
//   if(chassis.get_ForwardTracker_position()<45){
//     //Reset PID to ensure we dont die by trying to use a shit PID
//     setMotorVelocities(30);
//     wait(1,seconds);
//   }

//   //sum += timeTurnTest(135);
//   sum += timeDriveTest(-40 + chassis.get_ForwardTracker_position());

  

  return sum;
}


//This includes the actual PSO loops and tests for driving


void DrivePSO(const int num_particles, int num_interations, float max_velocity, float c1, float c2, float inches){
  logD("START PSO DRIVE", "numParticles:" + FTS(num_particles) + ", numInterations:" + FTS(num_interations) + ", maxVelocity:" + FTS(max_velocity) + ", c1:" + FTS(c1) + ", c2:" + FTS(c2) + ", inches:" + FTS(inches), true);
  
  std::string lastText2 = "";
  // std::array<float, num_particles> populationKP = [];
  // std::array<float, num_particles> populationKI = [];
  // std::array<float, num_particles> populationKD = [];
  // std::array<float, num_particles> populationSI = [];

  //Heres how this is going to go, We got KP, KI, KD and StartingI got it?
  float population[num_particles*4];
  float velocities[num_particles*4];

  //Best vlaues and best fitness scores for each member
  float pBest_params[num_particles*4];
  float pBest_values[num_particles];

  //Global best vlaues for all members, gBest_value needs to be very high
  float gBest_params[4];

  gBest_params[0] = float(1.5);
  gBest_params[1] = float(.0);
  gBest_params[2] = float(10);
  gBest_params[3] = float(0);
  //This also has to be low
  //float gBest_value = 3800;
  float gBest_value = 2000;//for sing,re turns

  float scaler = 0.25;

  for(int i = 0; i<(4*num_particles); i+=4){
    // population[i] = getRandomFloat(-scaler, scaler);
    // population[i+1] = getRandomFloat(-scaler, scaler);
    // population[i+2] = getRandomFloat(-scaler, scaler);
    // population[i+3] = getRandomFloat(-2.5 * scaler, 2.5 * scaler);

    population[i] = gBest_params[0] + getRandomFloat(-scaler/3, scaler/3);
    population[i+1] = gBest_params[1] + getRandomFloat(-scaler/7, scaler/7);
    population[i+2] = gBest_params[2] + getRandomFloat(-scaler, scaler);
    population[i+3] = gBest_params[3] + getRandomFloat(-2.5 * scaler, 2.5 * scaler);

    //   population[i] = gBest_params[0] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+1] = gBest_params[1] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+2] = gBest_params[2] * getRandomFloat(1-scaler, 1+scaler);
  //   population[i+3] = gBest_params[3] * getRandomFloat( 1 + (-2.5 * scaler), 1 + (2.5 * scaler));
  

    //Set up pbest to try to reduce beginning but keep score higher because we want variation
    pBest_values[i] = 8000;

    // pBest_params[i] = float(.4);
    // pBest_params[i+1] = float(.03);
    // pBest_params[i+2] = float(3);
    // pBest_params[i+3] = float(15);
  }


    for(int u = 0; u<num_interations; u+=1){   
      logD("PSO DRIVE GENERATION", "Interation:" + FTS(u) + ", GBEST Fitness:" + FTS(gBest_value) + ", GBEST KP:" + FTS(gBest_params[0]) + ", GBEST KI:" + FTS(gBest_params[1]) + ", GBEST KD:" + FTS(gBest_params[2]) + ", StartI:" + FTS(gBest_params[3]), true);
      for(int i = 0; i<(num_particles*4); i+=4){
        while(true){
          if((RightMiddleMotor.temperature(celsius)<55 && LeftMiddleMotor.temperature(celsius)<55) && (!Controller1.ButtonX.pressing())){
        float fitness = fullDriveTest(population[i], population[i+1], population[i+2], population[i+3]);

        std::string runInfo = ", Fitness:" + FTS(fitness) + ", KP:" + FTS(population[i]) + ", KI:" + FTS(population[i+1]) + ", KD:" + FTS(population[i+2]) + ", StartI:" + FTS(population[i+3]);
            std::string pBestInfo = ", PFitness:" + FTS(pBest_values[i/4]) + ", PKP:" + FTS(pBest_params[i]) + ", PKI:" + FTS(pBest_params[i+1]) + ", PKD:" + FTS(pBest_params[i+2]) + ", PStartI:" + FTS(pBest_params[i+3]);
            std::string gBestInfo = ", GFitness:" + FTS(gBest_value) + ", GKP:" + FTS(gBest_params[0]) + ", GKI:" + FTS(gBest_params[1]) + ", GKD:" + FTS(gBest_params[2]) + ", GStartI:" + FTS(gBest_params[3]);

        if (fitness < pBest_values[i]){
              pBest_params[i] = population[i];
              pBest_params[i+1] = population[i+1];
              pBest_params[i+2] = population[i+2];
              pBest_params[i+3] = population[i+3];
              pBest_values[i] = fitness;
        }

          // Update global best (gbest) position and value
          if (fitness < gBest_value) {
            //double check to make sure the values are actually better
            fitness = fullDriveTest(population[i], population[i+1], population[i+2], population[i+3]);
            if(fitness < gBest_value){
              gBest_params[0] = population[i];
              gBest_params[1] = population[i+1];
              gBest_params[2] = population[i+2];
              gBest_params[3] = population[i+3];
              gBest_value = fitness;
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

            std::string velocityInfo = ", vKP:" + FTS(velocities[i]) + ", vKI:" + FTS(velocities[i+1]) + ", vKD:" + FTS(velocities[i+2]) + ", vStartI:" + FTS(velocities[i+3]);
            logD("PSO DRIVE PARTICLE", "Interation:" + FTS(u) + ", Particle:" + FTS(i/4) +  runInfo + pBestInfo + gBestInfo + velocityInfo, true);
          }
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

        logD("CRITICAL OVERHEAT PSO DRIVE", firstLine + ":" + secondLine + ":" + thirdLine, true);
    
        wait(15,seconds);
      }


      }
    }
  }

  //Display output
  displayInfo(Controller1, floatToString(gBest_params[0]) + " " + floatToString(gBest_params[1]), floatToString(gBest_params[2]) + " " + floatToString(gBest_params[3]), floatToString(gBest_value));

  logD("STOP PSO DRIVE", floatToString(gBest_params[0]) + " " + floatToString(gBest_params[1]) + " " + floatToString(gBest_params[2]) + " " + floatToString(gBest_params[3]) + " " + floatToString(gBest_value), true);
}

