/**
 * @file InfinitePCA9685.h
 * @author Will Hickmott
 * @github TheSpaceEgg
 * @brief This file is part of the multiple instances PCA9685 library.
 * 
 * This code is provided under total free-use rights.
 * You are free to use, modify, and distribute this code without restriction.
 */

#ifndef multiPCA9685_H
#define multiPCA9685_H

#include <Wire.h> 
#include <vector>

// PCA9685 Register Definitions
#define MODE1 0x00	
#define MODE2 0x01
#define MODE1_RESTART 0x80 	
#define MODE1_SLEEP 0x10  
#define MODE1_AI 0x20  
#define PRESCALE 0xFE 
#define SERVO0 0x6			
#define MULTIPLIER 4	
#define PRE_SCALE 0xFE		
#define CLOCK_FREQ 27000000.0 // Standard for the PCA9685 module. Can be tweaked if necessary.

/**
 * @class PCA9685
 * @brief Class for controlling a single PCA9685 PWM driver over I2C.
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
     * @brief Sets the frequency of the PWM.
     * 
     * @param freq Frequency, usually 50 or 60Hz. Check device datasheets.
     */
	void setFreq(float freq);

    /**
     * @brief Initializes the PCA9685 driver and sets the frequency.
     * 
     * @param freq The PWM frequency.
     * @return True if initialization is successful.
     */
	bool begin(uint8_t freq);

    /**
     * @brief Resets the PCA9685 driver.
     */
	void reset(void);

    /**
     * @brief Sets the PWM output for a specific servo pin with custom ON/OFF times.
     * 
     * @param servo One of the PWM output pins, from 0 to 15.
     * @param on_value PWM ON time (0-4095).
     * @param off_value PWM OFF time (0-4095).
     */
	void setPWM(uint8_t servo, uint16_t on_value, uint16_t off_value);

    /**
     * @brief Sets the PWM output for a specific servo pin with default ON time (0).
     * 
     * @param servo One of the PWM output pins, from 0 to 15.
     * @param value PWM OFF time (0-4095).
     */
	void setPWM(uint8_t servo, uint16_t value);

    /**
     * @brief Gets the current PWM value of a specific servo pin.
     * 
     * @param servo One of the PWM output pins, from 0 to 15.
     * @return The current PWM value.
     */
    uint16_t getPWM(uint8_t servo);

private:
    TwoWire *i2c;  // Pointer to I2C bus (TwoWire instance)
    uint8_t deviceAddress;  // I2C address of the PCA9685 device
};

/**
 * @class MultiPCA9685
 * @brief Class for controlling multiple PCA9685 PWM drivers, either on different I2C buses or on a single bus with different addresses.
 */
class MultiPCA9685 {
public:

    /**
     * @brief Constructor for MultiPCA9685 across multiple I2C buses.
     * 
     * @param i2c_address_pairs A vector of TwoWire pointers and address pairs for the drivers.
     * @param frequency PWM frequency to be set for all drivers.
     */
    MultiPCA9685(const std::vector<std::pair<TwoWire*, uint8_t>>& i2c_address_pairs, uint8_t frequency = 50);

    /**
     * @brief Constructor for MultiPCA9685 on a single I2C bus with different addresses.
     * 
     * @param i2cBus The I2C bus (TwoWire instance) shared by all devices.
     * @param addresses A vector of I2C addresses for the PCA9685 devices.
     * @param frequency PWM frequency to be set for all drivers.
     */
    MultiPCA9685(TwoWire& i2cBus, const std::vector<uint8_t>& addresses, uint8_t frequency = 50);

    /**
     * @brief Sets the PWM signal for a specific motor or channel using custom ON/OFF times.
     * 
     * @param motorIndex Index of the motor/channel across all drivers.
     * @param on PWM signal ON time (0-4095).
     * @param off PWM signal OFF time (0-4095).
     */
    void setPWM(uint8_t motorIndex, uint16_t on, uint16_t off);

    /**
     * @brief Sets the PWM signal for a motor or channel using a single OFF value (ON time is 0).
     * 
     * @param motorIndex Index of the motor/channel across all drivers.
     * @param val PWM OFF time (0-4095).
     */
    void setPWM(uint8_t motorIndex, uint16_t val);

    /**
     * @brief Outputs the setup configuration for each PCA9685 driver, including bus number and I2C address.
     */
    void getSetup(void);

    /**
     * @brief Initializes all PCA9685 drivers.
     */
    void begin(void);

    /**
     * @brief Toggles the debug mode on or off.
     */
    void toggleDebug(void);

private:
    std::vector<PCA9685> pwmDrivers; 
    std::vector<std::pair<TwoWire*, uint8_t>> busAddressPairs;  
    uint8_t numDrivers;  
    uint16_t DevFrequency;  // Default frequency for all drivers
    const uint8_t outputsPerDriver = 16;  // Number of outputs per PCA9685 driver
};

#endif

