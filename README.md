# InfinitePCA9685 Library

## Description
The `InfinitePCA9685` library is designed to manage and control multiple PCA9685 devices using a single object in an Arduino environment. It simplifies the control of PWM devices (such as servos or LEDs) by abstracting the management of multiple I2C devices into a single controller class. This library is adapted for Arduino from the original [multiPCA9685 library](https://github.com/TheSpaceEgg/multiPCA9685) created for Linux environments.

## Author
- **Will Hickmott**  
  GitHub: [TheSpaceEgg](https://github.com/TheSpaceEgg)

## Features
- Control multiple PCA9685 devices via a single object.
- Set PWM frequency and individual PWM signals for each device.
- Easily scalable for complex hardware setups on Arduino.
- Debug mode to monitor the status of each device.
- Constructors based on multiple I2C buses or single bus.

## Dependencies
This library depends on the `Wire` library for I2C communication, which is standard in Arduino environments.

## Usage
To use the `InfinitePCA9685` library, include it in your Arduino sketch. See examples.