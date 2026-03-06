/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: Contains command functions, including:
 *              sendMotorCommand - outputs left and right PWM values over I2C to arduino motor controller
 *              handleUserCommand - handles command sent by user over bluetooth
 ************************************************************/

 #include <Arduino.h>

#include "commands.h"
#include <Wire.h>
#include "config.h"
#include "state.h"
#include "messenger.h"
#include "imu.h"
#include "debug.h"
#include "odometry.h"
#include "pid.h"
#include "iosfl.h"

// ==========================================================================

CommandType lastCommand = CMD_NONE;
int commandValue = 0;

// ==========================================================================

void sendMotorCommand(int16_t left, int16_t right) 
{
    static int16_t smoothLeft = 0;
    static int16_t smoothRight = 0;

    // SMOOTHING
    // ========================================================================
    const int rampStep = 10;   // Smaller = smoother, larger = faster response

    // // --- Smooth left ---
    // if (smoothLeft < left) {
    //     smoothLeft += rampStep;
    //     if (smoothLeft > left) smoothLeft = left;
    // } else if (smoothLeft > left) {
    //     smoothLeft -= rampStep;
    //     if (smoothLeft < left) smoothLeft = left;
    // }

    // // --- Smooth right ---
    // if (smoothRight < right) {
    //     smoothRight += rampStep;
    //     if (smoothRight > right) smoothRight = right;
    // } else if (smoothRight > right) {
    //     smoothRight -= rampStep;
    //     if (smoothRight < right) smoothRight = right;
    // }

    // // Clip range
    // if (smoothLeft > 200) smoothLeft = 200;
    // if (smoothLeft < -200) smoothLeft = -200;

    // if (smoothRight > 200) smoothRight = 200;
    // if (smoothRight < -200) smoothRight = -200;

    // SEND TO ARDUINO
    Wire.beginTransmission(UNO_ADDR);
    Wire.write((uint8_t*)&left, 2);
    Wire.write((uint8_t*)&right, 2);
    Wire.endTransmission();
}

// ==========================================================================

void handleUserCommand(String text) {
    text.toLowerCase();

    // ......................................................................
    // DRIVE FORWARD (PID CONTROL)
    // ......................................................................

if (text.startsWith("go")) {

    float speed = 0.0f;

    // Parse: forward 200   or   forward 0.3
    int parsed = sscanf(text.c_str(), "go %f", &speed);

    if (parsed == 1) {
        vLeft_target  = speed;
        vRight_target = speed;
        robotMode = MODE_DRIVE;

        sendLog("OK: driving forward at speed = " + String(speed));
    } else {
        sendLog("Usage: forward <speed>");
    }

    return;
}
    // ......................................................................
    // TURN
    // ......................................................................

    if (text.startsWith("turn")) {
        float angleDeg = text.substring(5).toFloat();     // get number after "turn "
        float angleRad = angleDeg * PI / 180.0f;

        targetHeading = wrapAngle(robotHeading + angleRad);

        robotMode = MODE_ROTATE;

        sendLog("Turning " + String(angleDeg) + " degrees.");
        return;
    }

    // ......................................................................
    // MANUAL DRIVE
    // ......................................................................

    if (text == "stop") {
        robotMode = MODE_IDLE;
        vLeft_target  = 0;
        vRight_target = 0;
        sendMotorCommand(0,0);
        sendLog("OK: stopping");
        return;
    }

    // ......................................................................
    // POSE / ZERO / CAL IMU
    // ......................................................................

    if (text == "pose") {
        sendLog(
            "Pose:\nX=" + String(robotX) +
            "\nY=" + String(robotY) +
            "\nHeading=" + String(robotHeading)
        );
        return;
    }

    // ........................................................................

    if (text == "zero") {
        robotX = 0;
        robotY = 0;
        robotHeading = 0;
        theta_enc = 0.0;
        theta_imu = 0;
        sendLog("Pose reset");
        return;
    }

    // ........................................................................

    if (text == "cal") {
        calibrateIMU();
        sendLog("IMU calibrated");
        return;
    }

    // ........................................................................
    // NAVIGATION -- IOSFL
    // ........................................................................

    if (text.startsWith("to")) {
        float gx, gy;
        int parsed = sscanf(text.c_str(), "to %f %f", &gx, &gy);
        if (parsed == 2) {
            goalX = gx;
            goalY = gy;
            robotMode = MODE_NAVIGATING;
            sendLog("Navigating to goal");
        } 
            else sendLog("Usage: to x y");
        
        return;
    }

    if (text == "report"){
        printDiagnostics();
        return;
    }

    if (text == "help"){
        sendLog("Available Commands:/n"
                "Go/n"
                "Stop/n"
                "Turn/n"
                "Goto x y/n"
                "Pose/n"
                "Zero/n"
                "Report/n");
    }

// PID tuning
if (text.startsWith("set pid")) {
    char param[8];
    float value;

    // expects "set pid kp 0.8"
    int parsed = sscanf(text.c_str(), "set pid %s %f", param, &value);

    if (parsed == 2) {

        if (strcmp(param, "kp") == 0) {
            Kp_left = value;
            Kp_right = value;
            sendLog("PID Kp set to " + String(value));
        }
        else if (strcmp(param, "ki") == 0) {
            Ki_left = value;
            Ki_right = value;
            sendLog("PID Ki set to " + String(value));
        }
        else if (strcmp(param, "kd") == 0) {
            Kd_left = value;
            Kd_right = value;
            sendLog("PID Kd set to " + String(value));
        }
        else {
            sendLog("Unknown PID term. Use kp | ki | kd");
        }

    } else {
        sendLog("Usage: set pid <kp|ki|kd> <value>");
    }

    return;
}

if (text == "get pid") {
    String msg =
        "PID Gains:\n"
        "Kp = " + String(Kp_left) + "\n" +
        "Ki = " + String(Ki_left) + "\n" +
        "Kd = " + String(Kd_left);

    sendLog(msg);
    return;
}


// iosfl tuning
if (text.startsWith("set iosfl")) {
    char param[8];
    float value;

    // expects "set iosfl k1 1.5" or "set iosfl k2 3.8"
    int parsed = sscanf(text.c_str(), "set iosfl %s %f", param, &value);

    if (parsed == 2) {
        if (strcmp(param, "k1") == 0) {
            k1 = value;
            sendLog("IOSFL k1 set to " + String(value));
        }
        else if (strcmp(param, "k2") == 0) {
            k2 = value;
            sendLog("IOSFL k2 set to " + String(value));
        }
        else {
            sendLog("Unknown IOSFL term. Use k1 | k2");
        }
    } else {
        sendLog("Usage: set iosfl <k1|k2> <value>");
    }
    return;
}

if (text == "get iosfl") {
    sendLog("IOSFL Gains:\n"
            "k1 = " + String(k1) + "\n" +
            "k2 = " + String(k2));
    return;
}


// ==================
// UNKNOWN COMMAND
// ==================
sendLog("Unknown command");

}
// ======================================================================================