/**
 * @file SingleBus.ino
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

// I2C addresses for PCA9685 devices on the same bus
std::vector<uint8_t> pcaAddresses = {0x40, 0x41};  // Add more addresses as required

// Instantiate the MultiPCA9685 object with the default device frequency of 50Hz on a single I2C bus
MultiPCA9685 manyPCA9685s(Wire, pcaAddresses, 50);

void setup() {
    Serial.begin(115200);

    // Initialize the I2C bus (standard Wire)
    Wire.begin(21, 22); // SDA = 21, SCL = 22 for ESP32 or adjust for your board

    // Initialize all PCA9685 devices
    manyPCA9685s.begin();
    //manyPCA9685s.toggleDebug(); // Enable debug output if needed

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
