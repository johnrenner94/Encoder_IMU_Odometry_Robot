/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

extern MPU6050 imu;

// Global accelerometer and gyro readings (in Gs and deg/sec)
extern float ax_g, ay_g, az_g;
extern float gx_dps, gy_dps, gz_dps;

// Initialize IMU hardware
void initIMU();

// Calibrate gyro biases (3-axis)
void calibrateIMU();

// Align IMU yaw to robot frame (set a heading zero reference)
void alignIMUToRobot(float robotHeadingAtCalibration);

// Read raw IMU data (populates global ax/ay/az, gx/gy/gz)
void readIMU();

// Update yaw integration (call every loop)
void updateIMUHeading(float dt);

// Filtered yaw-rate accessors
float getYawRate();     // filtered (rad/s)
float getRawYawRate();  // unfiltered (rad/s)

// Access integrated IMU heading (after alignment)
float getIMUHeading();


