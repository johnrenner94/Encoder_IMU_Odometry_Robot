/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once

// Define UNO I2C address
#define UNO_ADDR 0x08

// Define I2C Pins
#define IMU_SDA 21
#define IMU_SCL 22

// Define Encoder Pins
#define FL_A 16
#define FL_B 17
#define RL_A 18
#define RL_B 19
#define RR_A 33
#define RR_B 32
#define FR_A 27
#define FR_B 14

// Physical parameters
//   wheel diameter  = 2.5 in  = 0.0635 m
//   wheelbase       ≈ 5.25 in = 0.13335 m
//   ticks per rev   ≈ 960
//   circumference   = pi * d ≈ 0.19993 m

static const float wheelDiameter  = 0.0635f;          // m
static const float wheelCircum    = PI * wheelDiameter;
static const float ticksPerRev    = 960.0f;
const float metersPerTick  = wheelCircum / ticksPerRev; // ≈ 0.0002 m
static const float wheelbase      = 0.13335f;         // m
