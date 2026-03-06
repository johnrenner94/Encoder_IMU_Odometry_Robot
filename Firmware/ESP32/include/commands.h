/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

#pragma once
#include <Arduino.h>

enum CommandType {
    CMD_NONE,
    CMD_DRIVE_FORWARD,
    CMD_TURN,
    CMD_STOP,
    CMD_RESET_POSE,
    CMD_CAL_IMU,
    CMD_NAVIGATE_TO_GOAL
};

extern CommandType lastCommand;
extern int commandValue;   // Used for turn angle or drive speed etc.

void handleUserCommand(String text);
void sendMotorCommand(int16_t left, int16_t right);
