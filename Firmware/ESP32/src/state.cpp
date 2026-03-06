/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: State variable declaration and initialization
 ************************************************************/

 #include "state.h"

// ==========================
// Robot state (definition)
// ==========================

float robotX = 0.0f;
float robotY = 0.0f;
float robotHeading = 0.0f;   // radians

float vRight_target = 0.0f;
float vLeft_target = 0.0f;
float targetHeading = 0.0f;

int lastPwmL = 0;
int lastPwmR = 0;
float lastDt = 0.0f;
float lastDThetaEnc = 0.0f;

RobotMode robotMode = MODE_IDLE;