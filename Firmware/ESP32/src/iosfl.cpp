/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: contains code for IOSFL navigation control logic
 ************************************************************/

 #include <Arduino.h>
#include "state.h"
#include "config.h"
#include "odometry.h"

// Gains
float k1 = 1.0f;
float k2 = 5.0f;

// Goal pose
float goalX;
float goalY;

// Lookahead distance
const float L = 0.1f;  // 10 cm

void runIOSFL() {

    // Robot heading
    float hx = cosf(robotHeading);
    float hy = sinf(robotHeading);

    // Lookahead point
    float targetX = robotX + L * hx;
    float targetY = robotY + L * hy;

    // Compute error relative to lookahead point
    float ex = goalX - targetX;
    float ey = goalY - targetY;
    float e_dist = sqrtf(ex*ex + ey*ey);

    // Stop when close
    if (e_dist < 0.05f) {   // 5 cm tolerance
        vLeft_target  = 0;
        vRight_target = 0;
        robotMode = MODE_IDLE;
        return;
    }

    // In robot frame:

    float ex_r =  cosf(robotHeading)*ex + sinf(robotHeading)*ey;
    float ey_r = -sinf(robotHeading)*ex + cosf(robotHeading)*ey;


    // IOSFL control
    float v = k1 * ex_r;   // forward velocity (m/s)
    float w = k2 * ey_r;   // angular velocity (rad/s)

    // Convert to wheel speeds
    float vL = v - (wheelbase/2)*w;
    float vR = v + (wheelbase/2)*w;

    // Convert m/s → ticks/sec
    float vL_ticks = vL / metersPerTick;
    float vR_ticks = vR / metersPerTick;

    float W_MAX = 0.5f;   // instead of 1.0 or more
    w = constrain(w, -W_MAX, W_MAX);

    // Clamp
    float MAX_TICKS = 800;
    vLeft_target  = constrain(vL_ticks, -MAX_TICKS, MAX_TICKS);
    vRight_target = constrain(vR_ticks, -MAX_TICKS, MAX_TICKS);
}