#pragma once
#include <Arduino.h>

void initMotors();
void driveMotor(int pwmPin, int dirPin, int16_t speed);
