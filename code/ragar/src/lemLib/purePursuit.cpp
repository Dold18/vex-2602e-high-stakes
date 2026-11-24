#include <cmath>
#include <vector>
#include <string>

#include "botSetup/vex.h"
#include "DriveFuncs/motorFuncs.h" 
#include "info/settings.h"
#include "info/log.h"
#include "info/points.h"
#include "Libraries/controllerUI.h"
#include "Libraries/stringFunctions.h"
#include "Libraries/botErrorProcesses.h"

#include "autos/callbacks.h"
#include "Sensing/colorSort.h"

#include <functional>



float purePursuit(float Timeout, int currentPoint, int endPoint, int pointList, float lookaheadDist) {
    logD("START PURE", "timeout:" + FTS(Timeout) + 
         ", currentPoint:" + FTS(currentPoint) + 
         ", endPoint:" + FTS(endPoint) + 
         ", pointList:" + FTS(pointList));
    
    float startTime = Brain.timer(msec);
    
    // Determine endpoint if not specified
    if (endPoint == -1) {
        switch(pointList) {
            case 0: endPoint = length; break;
            case 1: endPoint = length1; break;
            case 2: endPoint = length2; break;
            case 3: endPoint = length3; break;
            case 4: endPoint = length4; break;
            case 5: endPoint = length5; break;
        }
    }

    // Initialize tracking variables
    float lastX = chassis.get_X_position();
    float lastY = chassis.get_Y_position();
    float prevVel = 0;
    
    while (currentPoint < endPoint-1 && (Brain.timer(msec) - startTime) < Timeout) {
        // Get current position
        float currentX = chassis.get_X_position();
        float currentY = chassis.get_Y_position();
        float currentHeading = to_rad(chassis.get_absolute_heading());
        
        // Get current and next path points
        float p1x, p1y, p2x, p2y;
        switch(pointList) {
            case 0: 
                p1x = pointx[currentPoint]; p1y = pointy[currentPoint];
                p2x = pointx[currentPoint+1]; p2y = pointy[currentPoint+1];
                break;
            case 1:
                p1x = pointx1[currentPoint]; p1y = pointy1[currentPoint];
                p2x = pointx1[currentPoint+1]; p2y = pointy1[currentPoint+1];
                break;
            // ... similar for other cases
        }
        
        // Calculate path segment vector
        float dx = p2x - p1x;
        float dy = p2y - p1y;
        
        // Calculate robot position relative to path segment
        float fx = p1x - currentX;
        float fy = p1y - currentY;
        
        // Calculate intersection using quadratic formula
        float a = dx*dx + dy*dy;
        float b = 2 * (fx*dx + fy*dy);
        float c = fx*fx + fy*fy - lookaheadDist*lookaheadDist;
        float discriminant = b*b - 4*a*c;
        
        float lookaheadX, lookaheadY;
        if (discriminant >= 0) {
            discriminant = sqrt(discriminant);
            float t1 = (-b - discriminant) / (2*a);
            float t2 = (-b + discriminant) / (2*a);
            
            // Choose the furthest valid intersection
            float t = -1;
            if (t2 >= 0 && t2 <= 1) t = t2;
            else if (t1 >= 0 && t1 <= 1) t = t1;
            
            if (t != -1) {
                lookaheadX = p1x + t*dx;
                lookaheadY = p1y + t*dy;
            } else {
                // No valid intersection, use end of segment
                lookaheadX = p2x;
                lookaheadY = p2y;
            }
        } else {
            // No intersection, use end of segment
            lookaheadX = p2x;
            lookaheadY = p2y;
        }
        
        // Calculate curvature
        float side = std::signbit(sin(currentHeading)*(lookaheadX - currentX) - 
                                cos(currentHeading)*(lookaheadY - currentY)) ? -1 : 1;
        float a_curv = -tan(currentHeading);
        float c_curv = tan(currentHeading)*currentX - currentY;
        float x_curv = fabs(a_curv*lookaheadX + lookaheadY + c_curv) / 
                      sqrt((a_curv*a_curv) + 1);
        float d_curv = hypot(lookaheadX - currentX, lookaheadY - currentY);
        float curvature = side * ((2 * x_curv) / (d_curv * d_curv));
        
        // Calculate target velocities
        float targetVel = 127; // Max speed, adjust as needed
        float leftVel = targetVel * (2 + curvature * chassis.getCenterDistance()) / 2;
        float rightVel = targetVel * (2 - curvature * chassis.getCenterDistance()) / 2;
        
        // Scale velocities to respect max speed
        float ratio = std::max(fabs(leftVel), fabs(rightVel)) / chassis.drive_max_voltage;
        if (ratio > 1) {
            leftVel /= ratio;
            rightVel /= ratio;
        }
        
        // Apply voltages
        chassis.drive_with_voltage(leftVel, rightVel);
        
        // Update tracking variables
        lastX = currentX;
        lastY = currentY;
        
        // Check if we should move to next point
        float distToEnd = hypot(p2x - currentX, p2y - currentY);
        if (distToEnd < lookaheadDist/2) {
            currentPoint++;
            displayInfo(Controller1, floatToString(currentPoint));
        }
        
        logD("LOG PURE POINT", "X: " + FTS(currentX) + 
             ", Y: " + FTS(currentY) + 
             ", LookaheadX: " + FTS(lookaheadX) + 
             ", LookaheadY: " + FTS(lookaheadY) + 
             ", Curvature: " + FTS(curvature) +
             ", LeftVel: " + FTS(leftVel) +
             ", RightVel: " + FTS(rightVel));
        
        wait(10,msec);
    }
    
    displayInfo(Controller1, "C4");
    logD("STOP PURE", "Ending pure loop", true);
    return Brain.timer(msec) - startTime;
}