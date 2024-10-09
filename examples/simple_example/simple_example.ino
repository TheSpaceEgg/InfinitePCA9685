/**
 * @file multi_example.ino
 * @author Will Hickmott
 * @github TheSpaceEgg
 * @brief This file is part of the multiple instances PCA9685 library.
 * 
 * This code is provided under total free-use rights.
 * You are free to use, modify, and distribute this code without restriction.
 */

#include <Wire.h>
#include "InfinitePCA9685.h"

// Get these values experimentally or from the datasheet for your servos
#define SERVOMIN    110
#define SERVOMAX    550

// Create TwoWire instances for ESP32's I2C buses
TwoWire I2C_1 = Wire;
TwoWire I2C_2 = Wire1;

// Vector of pairs of TwoWire pointers and I2C addresses
std::vector<std::pair<TwoWire*, uint8_t>> pcaBusAddressPairs = {
    { &I2C_1, 0x40 }, // I2C Bus 1, Address 0x40
    { &I2C_2, 0x41 }, // I2C Bus 2, Address 0x41
    { &I2C_1, 0x42 }, // I2C Bus 1, Address 0x42
    { &I2C_1, 0x48 }, // I2C Bus 1, Address 0x48
    { &I2C_2, 0x55 }, // I2C Bus 2, Address 0x55
    { &I2C_2, 0x80 }  // I2C Bus 2, Address 0x80
};

// Instantiate the MultiPCA9685 object with default device frequency of 50Hz
MultiPCA9685 manyPCA9685s(pcaBusAddressPairs);

void setup() {
    Serial.begin(115200);

    // Initialize the I2C buses (in this case using an ESP32 WROOM)
    I2C_1.begin(21, 22); // SDA = 21, SCL = 22 (for I2C bus 1)
    I2C_2.begin(25, 26); // SDA = 25, SCL = 26 (for I2C bus 2)

    // Check configuration of your setup
    manyPCA9685s.getSetup();
    /*
    This example should output (providing I2C buses are configured):
        MultiPCA9685 Configuration: 
        Number of PCA9685 Drivers: 6
        Device Frequency: 50 Hz

        Driver 1 -> I2C Bus: 1, Address: 0x40
        Driver 2 -> I2C Bus: 2, Address: 0x41
        Driver 3 -> I2C Bus: 1, Address: 0x42
        Driver 4 -> I2C Bus: 1, Address: 0x48
        Driver 5 -> I2C Bus: 2, Address: 0x55
        Driver 6 -> I2C Bus: 2, Address: 0x80
    */

    // Set PWM values for servo channels
    manyPCA9685s.setPWM(17, SERVOMIN);
    delay(1000); // delay in milliseconds
    manyPCA9685s.toggleDebug(); // Turns on debug output
    manyPCA9685s.setPWM(17, SERVOMAX);
    delay(1000);

    manyPCA9685s.setPWM(21, SERVOMIN);
    delay(1000);
    manyPCA9685s.setPWM(21, SERVOMAX);
    delay(1000);
    manyPCA9685s.toggleDebug(); // Turns off debug output

    manyPCA9685s.setPWM(95, SERVOMIN);
    delay(1000);
    manyPCA9685s.setPWM(95, SERVOMAX);
    delay(1000);
}

void loop() {
    // Main loop can be used to update servos, sensors, or other functionality as needed
}
