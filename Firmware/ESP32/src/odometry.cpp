/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: Contains functions for computing odometry from encoders and IMU
 ************************************************************/
#include <Arduino.h>
#include "odometry.h"
#include "state.h"
#include "encoders.h"
#include "imu.h"
#include "config.h"

float vLeft_obs  = 0.0f;
float vRight_obs = 0.0f;

float theta_enc = 0.0f;
float theta_imu = 0.0f;

float dtheta_enc = 0.0f;

float last_dCenter = 0.0f;
float last_dTheta  = 0.0f;

// ======================================================================

float wrapAngle(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

// ======================================================================

void initOdometry() {
    robotX = 0.0f;
    robotY = 0.0f;
    robotHeading = 0.0f;

    theta_enc = 0.0f;
    last_dCenter = 0.0f;
    last_dTheta  = 0.0f;

    vLeft_obs  = 0.0f;
    vRight_obs = 0.0f;
}

// ======================================================================

float computeDT() {                         // not being used
    static uint32_t last = millis();
    uint32_t now = millis();
    float dt = (now - last) / 1000.0f;
    if (dt < 0.0001f) dt = 0.0001f;
    last = now;
    return dt;
}

// ======================================================================

void updateOdometryFromEncoders(const EncoderDelta &enc, float dt) {
    float dL_ticks = enc.dLeft;
    float dR_ticks = enc.dRight;

    vLeft_obs  = dL_ticks / dt;
    vRight_obs = dR_ticks / dt;

    float dL = dL_ticks * metersPerTick;
    float dR = dR_ticks * metersPerTick;

    last_dCenter = 0.5f * (dL + dR);
    last_dTheta  = (dR - dL) / wheelbase;

    theta_enc = wrapAngle(theta_enc + last_dTheta);
}

// ======================================================================

void fuseIMUandOdometry(float dTheta_enc, float dt) {

    // Update encoder heading estimate ---
    theta_enc = wrapAngle(theta_enc + dTheta_enc);

    // Update IMU heading estimate ---
     theta_imu = wrapAngle(theta_imu + getYawRate() * dt);

    // Sensor fusion
    const float beta = 0.0f;
    float prevHeading = robotHeading;

    robotHeading = wrapAngle((1.0f - beta) * theta_imu + beta * theta_enc);

    // Compute fused delta rotation ---
    float dTheta_fused = wrapAngle(robotHeading - prevHeading);

    // Use fUSED heading to update position ---
    float headingMid = prevHeading + 0.5f * dTheta_fused;
    robotX += last_dCenter * cosf(headingMid);
    robotY += last_dCenter * sinf(headingMid);
}

