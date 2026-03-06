/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

#pragma once
#include <Arduino.h>
#include "encoders.h"

extern float vLeft_obs;
extern float vRight_obs;

extern float dtheta_enc;
extern float theta_enc;
extern float theta_imu;

extern float last_dCenter;
extern float last_dTheta;

void initOdometry();
float computeDT();

void updateOdometryFromEncoders(const EncoderDelta &enc, float dt);

void fuseIMUandOdometry(float dTheta_enc, float dt);

float wrapAngle(float a);