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

## Dependencies
This library depends on the `Wire` library for I2C communication, which is standard in Arduino environments.

## Usage
To use the `InfinitePCA9685` library, include it in your Arduino sketch. Here's an example of basic usage:

```cpp
#include <InfinitePCA9685.h>

void setup() {
    std::vector<std::pair<int, uint8_t>> devices = {{0x40}, {0x41}};
    MultiPCA9685 PCAobj(devices, 50);

    // Set PWM for the first motor on the first PCA9685
    PCAobj.setPWM(0, 0, 2000);
    // Set PWM for the first motor on the second PCA9685
    PCAobj.setPWM(15, 0, 2000);

    // Output current setup to serial
    PCAobj.getSetup();
}

void loop() {
    // Insert your repeated code here
}
