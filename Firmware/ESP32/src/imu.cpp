/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Descriptions: contains functions for calibrating and reading IMU
 ************************************************************/

#include <Arduino.h>
#include "imu.h"
#include <Wire.h>

#include "messenger.h"
#include "config.h"

// ======================================================================
// GLOBALS
// ======================================================================

MPU6050 imu;

// Raw accel in G's
float ax_g = 0;
float ay_g = 0;
float az_g = 0;

// Raw gyro in deg/sec
float gx_dps = 0;
float gy_dps = 0;
float gz_dps = 0;

// Gyro bias (deg/sec)
static float gyroBiasX = 0;
static float gyroBiasY = 0;
static float gyroBiasZ = 0;

// IMU→robot frame yaw alignment
static float yawOffset = 0;

// Heading and filtered yaw rate
static float imuHeading = 0;
static float filteredYawRate = 0;

// Filtering params
static const float yawDeadband = 0.02f;  // rad/s
static const float yawLpfAlpha = 0.20f;  // low-pass filter

// ======================================================================
// INIT
// ======================================================================

void initIMU() {
    Wire.begin(IMU_SDA, IMU_SCL);
    delay(100);

    imu.initialize();  // Rowberg function
    delay(200);

}

// ======================================================================
// 3-AXIS GYRO CALIBRATION
// ======================================================================

void calibrateIMU() {
    const int samples = 2000;

    Serial.println("Calibrating IMU... KEEP ROBOT STILL.");

    long sx = 0, sy = 0, sz = 0;
    delay(500);  // Let IMU settle

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        sx += gx;
        sy += gy;
        sz += gz;

        delayMicroseconds(500);
    }

    gyroBiasX = (float)sx / samples / 131.0f;
    gyroBiasY = (float)sy / samples / 131.0f;
    gyroBiasZ = (float)sz / samples / 131.0f;

    Serial.print("Gyro bias Z = ");
    Serial.println(gyroBiasZ);
}

// ======================================================================
// ALIGN IMU TO ROBOT FRAME
// ======================================================================

void alignIMUToRobot(float robotHeadingAtCalibration) {
    yawOffset = imuHeading - robotHeadingAtCalibration;
}

// ======================================================================
// READ ACCEL + GYRO RAW VALUES
// ======================================================================

void readIMU() {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Scale accel to G's
    ax_g = ax / 16384.0f;
    ay_g = ay / 16384.0f;
    az_g = az / 16384.0f;

    // Scale gyro to deg/sec
    gx_dps = gx / 131.0f;
    gy_dps = gy / 131.0f;
    gz_dps = gz / 131.0f;
}

// ======================================================================
// YAW RATE FUNCTIONS
// ======================================================================

// Raw yaw rate (rad/s)
float getRawYawRate() {
    return (gz_dps - gyroBiasZ) * PI / 180.0f;
}

// Filtered, deadbanded yaw rate (rad/s)
float getYawRate() {
    float rate = getRawYawRate();

    // Deadband small noise
    if (fabs(rate) < yawDeadband)
        rate = 0.0f;

    // Low-pass filter
    filteredYawRate = filteredYawRate + yawLpfAlpha * (rate - filteredYawRate);

    return filteredYawRate;
}