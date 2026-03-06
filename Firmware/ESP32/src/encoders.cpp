/************************************************************
 * File: 
 * Author: John Renner
 * School: University Of New Haven, Electrical and Computer Engineering
 * Course: ELEC6101 Autonomous Vehicles I -- Final Project (Fall 2025)
 * Date: 12/15/25
 * 
 * Description: Handles reading of encoders data using interrupts
 ************************************************************/

#include <Arduino.h>
#include "config.h"
#include "encoders.h"

// ==================================================================================== //

// initialize encoder counts

volatile long frCount = 0;    
volatile long flCount = 0;
volatile long rlCount = 0;
volatile long rrCount = 0;

static long lastLeftTicks = 0;
static long lastRightTicks = 0;

// ======================================================================================
// ENCODERS,       __    __    __
// forward: A:  __|  |__|  |__|  |__
//                __    __    __         .... wheels move forward if B leads A (fr FR, FL, and RR. RL plugged in backwards)
//          B: __|  |__|  |__|  |__

void encoderAttachTask(void *param) {

    attachInterrupt(digitalPinToInterrupt(FR_A), FR_ISR, RISING);  // attach interrupts to rising edge of channel A
    attachInterrupt(digitalPinToInterrupt(FL_A), FL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RL_A), RL_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RR_A), RR_ISR, RISING);

    vTaskDelete(NULL);  // kill task after attaching, FreeRTOS
}


// Define interrupts
void IRAM_ATTR FR_ISR() { frCount += (digitalRead(FR_B) ? 1 : -1); } // increase frcount if pin b is high, decrement if pin b is low
void IRAM_ATTR FL_ISR() { flCount += (digitalRead(FL_B) ? 1 : -1); }
void IRAM_ATTR RL_ISR() { rlCount += (digitalRead(RL_B) ? -1 : 1); }    // sign flipped, plugged in backwards
void IRAM_ATTR RR_ISR() { rrCount += (digitalRead(RR_B) ? 1 : -1); }


// ==================================================================================== //

void initEncoders() {

    pinMode(FR_A, INPUT_PULLUP); pinMode(FR_B, INPUT_PULLUP);
    pinMode(FL_A, INPUT_PULLUP); pinMode(FL_B, INPUT_PULLUP);
    pinMode(RL_A, INPUT_PULLUP); pinMode(RL_B, INPUT_PULLUP);
    pinMode(RR_A, INPUT_PULLUP); pinMode(RR_B, INPUT_PULLUP);

    // task to attach interrupts ON CORE 1
    xTaskCreatePinnedToCore(
        encoderAttachTask,      
        "encoderAttachTask",
        2048,       // stack size
        NULL,
        24,         // priority
        NULL,
        1           // core 1
    );

    lastLeftTicks  = 0;
    lastRightTicks = 0;
}

// ======================================================================================

EncoderDelta readEncoders() {

    long fl, rl, fr, rr;

    noInterrupts(); // turn off interrupts while reading
    fl = flCount;
    rl = rlCount;
    fr = frCount;
    rr = rrCount;
    interrupts();

    // average front and rear
    long leftTicks  = (fl + rl) / 2;
    long rightTicks = (fr + rr) / 2;

    // compute delta
    long dLeftTicks  = leftTicks  - lastLeftTicks;
    long dRightTicks = rightTicks - lastRightTicks;

    lastLeftTicks  = leftTicks;
    lastRightTicks = rightTicks;

    // clamp
    //if (abs(dLeftTicks) > 200)  dLeftTicks = 0;
    //if (abs(dRightTicks) > 200) dRightTicks = 0;

    EncoderDelta out;
    out.dLeft  = dLeftTicks;
    out.dRight = dRightTicks;

    return out;
}
// ======================================================================================