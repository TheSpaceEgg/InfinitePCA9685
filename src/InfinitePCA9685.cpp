/**
 * @file multiPCA9685.cpp
 * @author Will Hickmott
 * @github TheSpaceEgg
 * @brief This file is part of the multiple instances PCA9685 library.
 * 
 * This code is provided under total free-use rights.
 * You are free to use, modify, and distribute this code without restriction.
 */

#include "InfinitePCA9685.h"
#include <Arduino.h>>

/**
 * @brief Constructor for single PCA9685.
 * Initialises the I2C device and sets the device frequency.
 * 
 * @param i2cBus The TwoWire instance for the I2C bus.
 * @param deviceAddress The I2C address of the PCA9685.
 * @param freq The device frequency in Hz.
 */
PCA9685::PCA9685(TwoWire &i2cBus, uint8_t deviceAddress, uint8_t freq) {
    i2c = &i2cBus;  // Using TwoWire instead of custom I2C
    reset();
    setFreq(freq);
}

/**
 * @brief Resets the PCA9685 by writing to the MODE1 register.
 */
void PCA9685::reset() {
    i2c->beginTransmission(MODE1);
    i2c->write(0x20);  // Reset command
    i2c->endTransmission();
}

/**
 * @brief Sets the device frequency by calculating the prescale value
 * and writing it to the PRE_SCALE register.
 * 
 * @param freq The device frequency in Hz.
 */
void PCA9685::setFreq(uint8_t freq) {
    uint8_t prescale_val = static_cast<uint8_t>((CLOCK_FREQ / (4096 * freq)) - 1 + 0.5);
    i2c->beginTransmission(MODE1);
    i2c->write(0x10);  // Enter sleep mode to set prescaler
    i2c->endTransmission();

    i2c->beginTransmission(PRE_SCALE);
    i2c->write(prescale_val);  // Set prescaler value
    i2c->endTransmission();

    i2c->beginTransmission(MODE1);
    i2c->write(0xA0);  // Wake up and auto-increment
    i2c->endTransmission();
}

/**
 * @brief Sets the PWM signal for a specific servo with the default "on" time of 0.
 * 
 * @param servo The servo index (0-15).
 * @param value The "off" time value (0-4095).
 */
void PCA9685::setPWM(uint8_t servo, uint16_t value) {
	setPWM(servo, 0, value);
}

/**
 * @brief Sets the PWM signal for a specific servo with custom on and off times.
 * 
 * @param servo The servo index (0-15).
 * @param on_value The "on" time value (0-4095).
 * @param off_value The "off" time value (0-4095).
 */
void PCA9685::setPWM(uint8_t servo, uint16_t on_value, uint16_t off_value) {
    i2c->beginTransmission(SERVO0 + MULTIPLIER * servo);
    i2c->write(on_value & 0xFF);  // Low byte of ON
    i2c->write(on_value >> 8);    // High byte of ON
    i2c->write(off_value & 0xFF); // Low byte of OFF
    i2c->write(off_value >> 8);   // High byte of OFF
    i2c->endTransmission();
}

/**
 * @brief MultiPCA9685 constructor to handle multiple PCA9685 drivers.
 * Initializes each PCA9685 driver with the given TwoWire bus and address pairs.
 * 
 * @param bus_address_pairs Vector containing TwoWire bus pointers and device addresses.
 * @param frequency The device frequency in Hz for all devices.
 */
MultiPCA9685::MultiPCA9685(const std::vector<std::pair<TwoWire*, uint8_t>>& bus_address_pairs, uint8_t frequency)
  : busAddressPairs(bus_address_pairs), numDrivers(bus_address_pairs.size()), DevFrequency(frequency) {
    for (const auto& bus_address : bus_address_pairs) {
        TwoWire* i2cBus = bus_address.first;
        uint8_t address = bus_address.second;
        pwmDrivers.emplace_back(*i2cBus, address, frequency);
    }
}

/**
 * @brief Sets the PWM signal for a motor using the default on time of 0.
 * 
 * @param motorIndex The index of the motor across all drivers.
 * @param val The "off" time value (0-4095).
 */
void MultiPCA9685::setPWM(uint8_t motorIndex, uint16_t val) {
  setPWM(motorIndex, 0, val);
}

/**
 * @brief Sets the PWM signal for a motor with custom on and off times, across multiple PCA9685 drivers.
 * If debug mode is enabled, it prints the details of the command.
 * 
 * @param motorIndex The index of the motor across all drivers.
 * @param on The "on" time value (0-4095).
 * @param off The "off" time value (0-4095).
 */
void MultiPCA9685::setPWM(uint8_t motorIndex, uint16_t on, uint16_t off) {
    uint8_t driverIndex = motorIndex / outputsPerDriver;
    uint8_t channel = motorIndex % outputsPerDriver;
    if (driverIndex < numDrivers) {
        auto& driver = pwmDrivers[driverIndex];
        auto& bus_address = busAddressPairs[driverIndex];
        if (PCA9685_DEBUG) {
            Serial.print("Setting PWM (");
            Serial.print(on);
            Serial.print(", ");
            Serial.print(off);
            Serial.print(") for motor ");
            Serial.print(motorIndex);
            Serial.print(" on board ");
            Serial.print(driverIndex);
            Serial.print(", channel ");
            Serial.print(channel);
            Serial.print(" at address 0x");
            Serial.print(bus_address.second, HEX);
            Serial.println(" on I2C custom bus ");
        }
        driver.setPWM(channel, on, off);
    }
}

/**
 * @brief Outputs the setup configuration for each PCA9685 driver, including bus number and I2C address.
 */
void MultiPCA9685::getSetup(void) {
    Serial.println("\n\t/*** MultiPCA9685 Configuration: ***/");
    Serial.print("\tNumber of PCA9685 Drivers: ");
    Serial.println(numDrivers);
    Serial.print("\tDevice Frequency: ");
    Serial.print(DevFrequency);
    Serial.println(" Hz\n");

    for (size_t i = 0; i < busAddressPairs.size(); ++i) {
        TwoWire* i2cBus = busAddressPairs[i].first;
        uint8_t address = busAddressPairs[i].second;
        Serial.print("\tDriver ");
        Serial.print(i + 1);
        Serial.print(" -> I2C Bus: ");
        Serial.print((uintptr_t)i2cBus);
        Serial.print(", Address: 0x");
        Serial.print(address, HEX);
        Serial.println();
    }
}

/**
 * @brief Toggles the debug mode on or off. When debug mode is on, it outputs information about the PWM settings.
 */
void MultiPCA9685::toggleDebug() {
    PCA9685_DEBUG = !PCA9685_DEBUG;
    if (PCA9685_DEBUG) {
        Serial.println("Debug output On.");
    } else {
        Serial.println("Debug output Off.");
    }
}

