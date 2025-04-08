#ifndef ToFSensor_h
#define ToFSensor_h

#include <Arduino.h>
#include "Global.h"
#include <Wire.h>

#include "Adafruit_VL6180X.h"

#define MULTIPLEXER_I2C_ADDRESS 0x70

/**
 * A class which represents all sensors for this application.
 */
class ToFSensor {
  public:
    /***
     * Constructor - initialises the Adafruit_VL6180X object.
     */
    ToFSensor();

    /**
     * Sets the I2C pins to use the second I2C bus and calls Wire.begin().
     */
    void begin();

    /**
     * Returns the range reading for sensor or 255 if there is an error.
     * Returns -1 if sensor is not connected.
     */
    int read(uint8_t sensor);
  
  private:
    Adafruit_VL6180X vl;

};

#endif
