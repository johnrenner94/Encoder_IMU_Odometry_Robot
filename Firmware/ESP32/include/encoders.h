/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 ************************************************************/

 #pragma once

#include <Arduino.h>
#include "config.h"

extern volatile long frCount;
extern volatile long flCount;
extern volatile long rlCount;
extern volatile long rrCount;

struct EncoderDelta{
    long dLeft;
    long dRight;
};

void initEncoders();
EncoderDelta readEncoders();

void IRAM_ATTR FL_ISR();
void IRAM_ATTR FR_ISR();
void IRAM_ATTR RR_ISR();
void IRAM_ATTR RL_ISR();


