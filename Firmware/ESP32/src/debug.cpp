/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: contains functions for printing debug info to serial monitor and over BT connection
 ************************************************************/

 #include "state.h"   // gives access to globals
#include "debug.h"
#include "messenger.h"
#include "odometry.h"
#include "pid.h"
#include "iosfl.h"

// ======================================================================================

void printDiagnostics() {

    String msg =
        "Diag:\n"
        "Mode=" + String(robotMode) + "\n" +
        "Pose: (" + String(robotX, 3) + ", " + String(robotY, 3) + ")\n" +
        "Heading=" + String(robotHeading, 3) + "\n" +
        "Goal: (" + String(goalX, 3) + ", " + String(goalY, 3) + ")\n"
        "vTgt L/R: " + String(vLeft_target, 3) + " / " + String(vRight_target, 3) + "\n" +
        "vObs L/R: " + String(vLeft_obs, 3) + " / " + String(vRight_obs, 3) + "\n" +
        "PWM L/R: " + String(lastPwmL) + " / " + String(lastPwmR) + "\n" +
        "dt=" + String(lastDt, 4) + "\n"

        "PID Left:  Kp=" + String(Kp_left) +
        " Ki=" + String(Ki_left) +
        " Kd=" + String(Kd_left) + "\n" +

        "PID Right: Kp=" + String(Kp_right) +
        " Ki=" + String(Ki_right) +
        " Kd=" + String(Kd_right) + "\n" +

        "IOSFL: k1=" + String(k1) +
        " k2=" + String(k2) + "\n";

    // Choose your output transport here:
    // Serial.println(msg);
    sendLog(msg);  // your existing WiFi/Bluetooth log function
}

// ======================================================================================

void printDiagnosticsSerial() {

    Serial.println("=== Diagnostics ===");
    Serial.print("Mode = ");
    Serial.println(robotMode);

    Serial.print("vLeft_target  = ");
    Serial.println(vLeft_target);

    Serial.print("vRight_target = ");
    Serial.println(vRight_target);

    Serial.print("vLeft_obs     = ");
    Serial.println(vLeft_obs);

    Serial.print("vRight_obs    = ");
    Serial.println(vRight_obs);

    Serial.print("PWM_L         = ");
    Serial.println(lastPwmL);

    Serial.print("PWM_R         = ");
    Serial.println(lastPwmR);

    Serial.print("dt            = ");
    Serial.println(lastDt, 6);    // high-resolution dt

    Serial.print("dtheta_enc    = ");
    Serial.println(lastDThetaEnc, 6);

    Serial.print("Pose: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.print(robotY);
    Serial.print(", H=");
    Serial.print(robotHeading);
    Serial.println(")");

    Serial.println("=====================");
}

// ======================================================================