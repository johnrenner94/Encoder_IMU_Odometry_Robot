/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: contains PID control logic for wheel speed
 ************************************************************/

#include <Arduino.h>
#include "pid.h"

// ==========================================================================

// PID Gains
float Kp_left = 0.1f;
float Ki_left = 0.00f;
float Kd_left = 0.00f;

float Kp_right = 0.1f;
float Ki_right = 0.00f;
float Kd_right = 0.00f;

//==========================================================================

PIDState pidLeft = {0,0,false};
PIDState pidRight = {0,0,false};

// ==========================================================================

void initPID(){
    pidLeft = {0,0,false,0};
    pidRight = {0,0,false,0};
}

// ==========================================================================

float runPID(float v_target, float v_obs, float dt,
             float Kp, float Ki, float Kd, PIDState &state)
{
    float error = v_target - v_obs;

    // integral
    state.integral += error * dt;
    // state.integral = constrain(state.integral, -500.0f, 500.0f); // Anti-windup

    // Derivative - filtered
    float derivative = 0.0f;

    if (state.initialized) {
        float rawD = (error - state.prevError) / dt;
        derivative = 0.7f * state.derivativePrev + 0.3f * rawD;
    }
    state.derivativePrev = derivative;

    state.initialized = true;
    state.prevError = error;

    float output = Kp*error + Ki*state.integral + Kd*derivative;

    // clamp
    if (output < -255) output = -255;
    if (output > 255) output = 255;

    Serial.printf("target=%.2f obs=%.2f err=%.2f pwm=%.2f\n",
              v_target, v_obs, error, output);

    return output;
}


//=============================================================== 

void computeWheelPWM(float vLeft_target, float vRight_target,
                     float vLeft_obs, float vRight_obs, float dt,
                     int16_t &pwmL, int16_t &pwmR) {

    // feedforward
    int baseL = pwmFromSpeed(vLeft_target);
    int baseR = pwmFromSpeed(vRight_target);

    // pid correction
    float correctionL = runPID(vLeft_target, vLeft_obs, dt,
                               Kp_left, Ki_left, Kd_left, pidLeft);

    float correctionR = runPID(vRight_target, vRight_obs, dt,
                               Kp_right, Ki_right, Kd_right, pidRight);

    // correction scale
    const float CORRECTION_SCALE = 0.3f;

    pwmL = baseL + CORRECTION_SCALE * correctionL;
    pwmR = baseR + CORRECTION_SCALE * correctionR;

    // deadband
     if (abs(pwmL) < 100 && pwmL != 0) pwmL = (pwmL > 0 ? 100 : -100);
     if (abs(pwmR) < 100 && pwmR != 0) pwmR = (pwmR > 0 ? 100 : -100);

    // clamp
    pwmL = constrain(pwmL, -255, 255);
    pwmR = constrain(pwmR, -255, 255);
}
 
// ==============================================================================

int pwmFromSpeed(float v_target) {
    // Rough scaling — needs tuning
    if (v_target == 0) return 0;

    const float slope = 0.2f;   // PWM gained per tick/sec
    const int deadband = 0;     // robot won't move below ~30 PWM

    int pwm = slope * v_target;
    if (pwm > 255) pwm = 255;
    return pwm;
}

// ===============================================================================

void setPIDLeft(float kp, float ki, float kd) {
    Kp_left = kp; Ki_left = ki; Kd_left = kd;
}

void setPIDRight(float kp, float ki, float kd) {
    Kp_right = kp; Ki_right = ki; Kd_right = kd;
}

// ==============================================================================

