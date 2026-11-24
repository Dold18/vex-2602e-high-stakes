#ifndef PSOTURN_H
#define PSOTURN_H

#include "JAR-Template/drive.h"





int timeTurnTest(float angle);

float fullTurnTest(float KP, float KI, float KD, float StartingI);

std::string getPSOPopulation(const int num_particles, int angle);

void TurningPSO(const int num_particles, int num_interations, float max_velocity, float c1, float c2, float angle);

void PSOLoop(int num_interations, int num_particles, std::string popData, float angle, float max_velocity, float c1, float c2, std::string SAVEFILE);

#endif