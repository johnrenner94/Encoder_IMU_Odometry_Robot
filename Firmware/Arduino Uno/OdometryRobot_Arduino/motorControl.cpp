#include "motorControl.h"
#include "config.h"

void initMotors() {
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
}

void driveMotor(int pwmPin, int dirPin, int16_t speed) {
    speed = constrain(speed, -255, 255);

    if (speed >= 0) {
        digitalWrite(dirPin, HIGH);
        analogWrite(pwmPin, speed);
    } else {
        digitalWrite(dirPin, LOW);
        analogWrite(pwmPin, -speed);
    }
}
