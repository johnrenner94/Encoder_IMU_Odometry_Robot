#include <Wire.h>
#include "i2c_slave.h"
#include "config.h"

int16_t leftSpeed  = 0;
int16_t rightSpeed = 0;

void receiveEvent(int count) {
    if (count < 4) return;

    uint8_t L0 = Wire.read();
    uint8_t L1 = Wire.read();
    uint8_t R0 = Wire.read();
    uint8_t R1 = Wire.read();

    leftSpeed  = (int16_t)((L1 << 8) | L0);
    rightSpeed = (int16_t)((R1 << 8) | R0);

    Serial.print("Received L=");
    Serial.print(leftSpeed);
    Serial.print("  R=");
    Serial.println(rightSpeed);
}

void initI2CSlave() {
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
}
