/**
 * @file multiPCA9685.h
 * @author Will Hickmott
 * @github TheSpaceEgg
 * @brief This file is part of the multiple instances PCA9685 library.
 * 
 * This code is provided under total free-use rights.
 * You are free to use, modify, and distribute this code without restriction.
 */

#ifndef multiPCA9685_H
#define multiPCA9685_H

#include <Wire.h> // Updated for TwoWire usage
#include <vector>

#define MODE1 0x00			
#define SERVO0 0x6			
#define MULTIPLIER 4	
#define PRE_SCALE 0xFE		
#define CLOCK_FREQ 27000000.0 // standard for the PCA9685 module. Can be tweaked.


/**
 * @class PCA9685
 * @brief Class for controlling a single PCA9685 PWM driver over I2C. Very light, much like the Adafruit.
 */

class PCA9685 {
public:
    /**
     * @brief Constructor for single PCA9685.
     * 
     * @param i2cBus TwoWire instance (I2C bus)
     * @param deviceAddress I2C address of the PCA9685.
     * @param freq PWM frequency to be set for the driver.
     */
    PCA9685(TwoWire &i2cBus, uint8_t deviceAddress, uint8_t freq);
    
    /**
     * @brief Sets the frequency of the servo.
     * 
     * @param freq will usually be 50 or 60. Check device datasheets.
     */
	void setFreq(uint8_t freq);

    /**
     * @brief  Sets the PWM output of one of the PCA9685 pins
     * 
     * @param servo One of the PWM output pins, from 0 to 15
     * @param on when in the 4096-part cycle to turn the PWM output ON
     * @param off when in the 4096-part cycle to turn the PWM output OFF
     */
	void setPWM(uint8_t servo, uint16_t on_value, uint16_t off_value);

    /**
     * @brief  Sets the PWM output of one of the PCA9685 pins
     * 
     * @param servo One of the PWM output pins, from 0 to 15
     * @param value when in the 4096-part cycle to turn the PWM output ON
     */
	void setPWM(uint8_t servo, uint16_t value);

    /**
     * @brief  Should return the PWM value of a channel - PCA issues means this does not work!
     */
    uint16_t getPWM(uint8_t servo);
private:
    TwoWire *i2c;  // Changed from I2CDevice to TwoWire for Arduino compatibility
	void reset(void); // Quick toggle resets the device.
};

/**
 * @class MultiPCA9685
 * @brief Class for controlling multiple PCA9685 PWM drivers across different I2C buses.
 */
class MultiPCA9685 {
public:

    /**
     * @brief Constructor for MultiPCA9685.
     * 
     * @param bus_address_pairs A vector of TwoWire pointers and address pairs for the drivers.
     * @param frequency PWM frequency to be set for all drivers.
     */
    MultiPCA9685(const std::vector<std::pair<TwoWire*, uint8_t>>& i2c_address_pairs, uint8_t frequency = 50);

    /**
     * @brief Sets the PWM signal for a specific motor or channel.
     * 
     * @param motorIndex Index of the motor/channel across all drivers.
     * @param on PWM signal on-time (0-4095).
     * @param off PWM signal off-time (0-4095).
     */
    void setPWM(uint8_t motorIndex, uint16_t on, uint16_t off);

    /**
     * @brief Sets the PWM signal for a motor or channel using a single value.
     * 
     * @param motorIndex Index of the motor/channel across all drivers.
     * @param val PWM pulse width (0-4095).
     */
    void setPWM(uint8_t motorIndex, uint16_t val);

    void getSetup(void);

    void toggleDebug(void);

    bool PCA9685_DEBUG = false; // set true to output some debugging info.

private:
    std::vector<PCA9685> pwmDrivers;
    std::vector<std::pair<TwoWire*, uint8_t>> busAddressPairs;
    uint8_t numDrivers;
    uint16_t DevFrequency; // default is 50. Check servo datasheet.
    const uint8_t outputsPerDriver = 16; // How many channels on each PCA9685 board need considering.
};

#endif

