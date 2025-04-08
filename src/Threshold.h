#ifndef Threshold_h
#define Threshold_h

#include <Arduino.h>
#include "Global.h"

enum class State { Unknown, Near, Far };

typedef struct {
  uint8_t sensor;
  uint8_t threshold;
  uint8_t valueNear;
  uint8_t valueFar;
  uint16_t eventIndexNear;
  uint16_t eventIndexFar;
  State currentState;
} threshold_t;

typedef struct { 
  struct {
    threshold_t threshold[NUM_THRESHOLD];
  } sensor[NUM_SENSOR];
} ThresholdArray_t; 

/**
 * A class which contains all the thresholds for this application.
 */
class Threshold {
  public:
    /**
     * Stores the threshold in the array of thresholds.
     * Called at startup and when config value changed.
     */
    void set(threshold_t threshold);

    /**
     * Compares range with the current state for this threshold for this sensor and
     * returns the index of the event to send or -1 if there is no event to be sent.
     */
    int check(uint8_t sensor, uint8_t threshold, int range);

    /**
     * Returns the current state for the threshold which contains index.
     */
    State getStateForEventIndex(uint16_t index);

    /**
     * Displays all sensor information to the serial port.
     */
    void print();

  private:
    ThresholdArray_t thresholds;
};

#endif
