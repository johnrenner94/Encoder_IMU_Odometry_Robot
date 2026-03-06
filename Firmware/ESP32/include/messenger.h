/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once

#include <Arduino.h>

// Initializes Bluetooth Low Energy (BLE) UART-style link.
// (Function name kept as initWifi for compatibility with existing code.)
void initBluetooth();

// Called from loop(); for BLE this is currently a no-op but kept for
// compatibility with the existing main loop structure.
void wifiUpdate();

// Send a log/status string to both Serial and (if connected) over BLE
// to the paired phone app.
void sendLog(const String &msg);
