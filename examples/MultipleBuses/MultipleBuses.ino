/**
 * @file MultipleBuses.ino
 * @author Will Hickmott
 * @github TheSpaceEgg
 * @brief This file is part of the multiple instances PCA9685 library.
 * 
 * This code is provided under total free-use rights.
 * You are free to use, modify, and distribute this code without restriction.
 */

#include <Wire.h>
#include "InfinitePCA9685.h"

// Define the minimum and maximum pulse length values for your servos
#define SERVOMIN    200  // Minimum pulse length count (out of 4096)
#define SERVOMAX    400  // Maximum pulse length count (out of 4096)

// Create TwoWire instances for ESP32's I2C buses
TwoWire I2C_1 = Wire;
TwoWire I2C_2 = Wire1;

// Vector of pairs of TwoWire pointers and I2C addresses
std::vector<std::pair<TwoWire*, uint8_t>> pcaBusAddressPairs = {
    { &I2C_1, 0x40 }, // I2C Bus 1, Address 0x40
    { &I2C_2, 0x41 }, // I2C Bus 2, Address 0x41
};

// Instantiate the MultiPCA9685 object with default device frequency of 50Hz
MultiPCA9685 manyPCA9685s(pcaBusAddressPairs);

void setup() {
    Serial.begin(115200);

    // Initialize I2C buses
    I2C_1.begin(21, 22); // SDA = 21, SCL = 22 (for I2C bus 1)
    I2C_2.begin(33, 32); // SDA = 33, SCL = 32 (for I2C bus 2)

    // Initialize PCA9685 devices
    manyPCA9685s.begin();
    //manyPCA9685s.toggleDebug(); // Enable debug output

    // Display the current setup configuration
    manyPCA9685s.getSetup();

}

void loop() {
    // Pulse servos 0-31 from SERVOMAX to SERVOMIN
    for (uint8_t servo = 0; servo < 32; servo++) {
        Serial.print("Pulsing servo: ");
        Serial.println(servo);

        // Move servo to max position
        manyPCA9685s.setPWM(servo, SERVOMAX);
        delay(500); // Short delay between movements

        // Move servo back to min position
        manyPCA9685s.setPWM(servo, SERVOMIN);
    }

    // Add a delay before starting the sequence again
    delay(2000);
}