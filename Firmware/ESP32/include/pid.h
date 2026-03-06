/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once

#include <Arduino.h>

extern float Kp_left;
extern float Ki_left;
extern float Kd_left;

extern float Kp_right;
extern float Ki_right;
extern float Kd_right;

struct PIDState {
    float prevError;
    float integral;
    bool initialized;
    float derivativePrev;   // <-- Add this
};


extern PIDState pidLeft;
extern PIDState pidRight;

void initPID();

float runPID(float v_target,  float vLeft_obs, 
             float dt, float Kp, float Ki, float Kd, PIDState &state);

void computeWheelPWM(float vLeft_target, float vRight_target,
                     float vLeft_obs, float vRight_obs,
                     float dt,
                     int16_t &pwmL, int16_t &pwmR);

void setPIDLeft(float kp, float ki, float kd);
void setPIDRight(float kp, float ki, float kd);
int pwmFromSpeed(float v_target);