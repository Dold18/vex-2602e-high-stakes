#ifndef PIDD_H
#define PIDD_H

#include <functional>
 

void call4THC(float angle, int excTime, std::function<void()> callback);
void call4DD(float distance, double excTime, std::function<void()> callback, float chainDistBreak);

void call4DD(float distance, double excTime, std::function<void()> callback);

void THC(float angle);
void donaldDuck(float distance);
void donaldDuck(float distance, float chainBreak);

#endif