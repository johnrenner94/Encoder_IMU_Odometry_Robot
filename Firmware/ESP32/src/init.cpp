/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: contains initialization function for main code
 ************************************************************/

 #include <Arduino.h>

#include "init.h"
#include "messenger.h"
#include "encoders.h"
#include "imu.h"
#include "odometry.h"

#include <Wire.h>

// ======================================================================

void initializeRobot() {

    Serial.begin(115200);
    delay(250);
    Serial.println("Serial Monitor Online");

    initBluetooth();
    delay(1000);

    sendLog("Initializing Encoders...");
    initEncoders();
    delay(500);

    sendLog("Connecting to IMU...");
    initIMU();
    delay(500);

    if (!imu.testConnection()) {
        sendLog("ERROR: MPU6050 NOT CONNECTED!");
    } else {
        sendLog("IMU connected!");
    }

    delay(500);

    sendLog("Calibrating IMU...");
    calibrateIMU();
    delay(500);

    sendLog("Starting Odometry...");
    initOdometry();
    delay(500);

    sendLog("Ready!");
}
// ======================================================================