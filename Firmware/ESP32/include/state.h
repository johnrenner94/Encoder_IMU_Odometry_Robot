/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once

// Robot position (global coordinate frame, in meters)
extern float robotX;
extern float robotY;

// Robot orientation (heading in radians or degrees)
extern float robotHeading;
extern float targetHeading;

extern float vLeft_target;
extern float vRight_target;

extern float goalX;
extern float goalY;

extern int lastPwmL;
extern int lastPwmR;
extern float lastDt;
extern float lastDThetaEnc;


enum RobotMode{
    MODE_IDLE,
    MODE_DRIVE,
    MODE_NAVIGATING,
    MODE_ROTATE
};

extern RobotMode robotMode;