/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: Position estimation robot using wheel encoders and 6 axis accelerometer/gyroscope
 *              Robot takes user commands via bluetooth interface
 *              Tracks x, y, and heading using encoder counts and z-axis angular velocity 
 *              Outputs diagnostic/debug log over BT connection
 *              
 *              Main code includes system inititialization, main control loop (sensing, estimation, state handling) 
 *              plus motor control and logging
 * 
 *              This code was written originally in the Arduino IDE, and later moved to Visual Studio.
 *              It uses the PlatformIO extension and includes the following libraries:
 * 
 *                                      witnessmenow/UniversalTelegramBot@^1.3.0
 *                                      bblanchon/ArduinoJson@^6.21.0
 *                                      Wire
 *                                      ESP32 BLE Arduino
 * 
 *              
 * 
 ************************************************************/

#include <Arduino.h>

#include "config.h"
#include "encoders.h"
#include "imu.h"
#include "debug.h"
#include "messenger.h"
#include "state.h"
#include "odometry.h"
#include "commands.h"
#include "init.h"
#include "pid.h"
#include "iosfl.h"

static int16_t pwmL = 0;
static int16_t pwmR = 0;

// ======================================================================

void setup() {

    initializeRobot();

}

// ======================================================================

void loop() {

    uint32_t now = millis();

    // control loop timer
    const unsigned long CONTROL_PERIOD_MS = 10;
    static uint32_t lastControl = millis();

    if (now - lastControl >= CONTROL_PERIOD_MS) {

        // 1. Compute dt
        // ======================================================================
        float dt = (now - lastControl) / 1000.0f;
        lastControl = now;

        // 2. SENSOR INPUT
        // ======================================================================
        readIMU();                                  // gets ax_g ay_g az_g / gx gy gz                         
        EncoderDelta delta = readEncoders();        // 

        // 3. ODOMETRY
        // ======================================================================
        updateOdometryFromEncoders(delta, dt);
        fuseIMUandOdometry(dtheta_enc, dt);

        Serial.printf("target=%f  vL_obs=%f  vR_obs=%f\n", 
        vLeft_target, vLeft_obs, vRight_obs);

        // 4. STATE MACHINE
        // ======================================================================

        switch (robotMode) {

            // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

            case MODE_IDLE:
                vLeft_target  = 0;
                vRight_target = 0;
                pwmL = 0;
                pwmR = 0;
                break;

            // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

            case MODE_DRIVE:
                break;

            // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

            case MODE_NAVIGATING:
            {
                runIOSFL();

                // goal detection
                float dx = goalX - robotX;
                float dy = goalY - robotY;
                float dist = sqrt(dx*dx + dy*dy);

                if (dist < 0.10f) {  // within 10 cm
                    sendLog("Goal reached");
                    robotMode = MODE_IDLE;
                    vLeft_target  = 0;
                    vRight_target = 0;
                }
                break;
            }

            // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

            case MODE_ROTATE:

                float error = wrapAngle(targetHeading - robotHeading);

                // --- Stop condition ---
                if (fabs(error) < 0.05f) {   // ~3 degrees
                    sendMotorCommand(0, 0);
                    robotMode = MODE_IDLE;
                    sendLog("Turn complete.");
                    return;
                }

                // --- Proportional heading controller ---
                float kTurn = 600.0f;  

                // Desired wheel speeds from heading error
                vLeft_target  = -kTurn * error;
                vRight_target =  kTurn * error;

                    break;
            }

        // 5. MOTOR CONTROL ===================================================

            computeWheelPWM(vLeft_target, vRight_target, vLeft_obs, vRight_obs, dt, pwmL, pwmR);

            if (robotMode == MODE_IDLE){
                pwmL = 0;
                pwmR = 0;
            }

            sendMotorCommand(pwmL, pwmR);
        
        // 6. LOGGING STORAGE
        // ======================================================================

        lastDt = dt;
        lastPwmL = pwmL;
        lastPwmR = pwmR;
        lastDThetaEnc = dtheta_enc;

        // DIAGNOSTICS (EVERY 250ms) 
        static uint32_t nextDiag = 0;
        if (now >= nextDiag) {
            nextDiag = now + 250;    // every 250ms (4 Hz)
            printDiagnostics();
        }

    }

}