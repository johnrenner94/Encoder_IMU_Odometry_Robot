#include <Arduino.h>

#include "config.h"
#include "motorControl.h"
#include "i2c_slave.h"

void setup() {
    Serial.begin(115200);

    initMotors();
    initI2CSlave();

    Serial.println("UNO motor receiver ready");
}

void loop() {
    driveMotor(LEFT_PWM, LEFT_DIR, leftSpeed);
    driveMotor(RIGHT_PWM, RIGHT_DIR, rightSpeed);
}
