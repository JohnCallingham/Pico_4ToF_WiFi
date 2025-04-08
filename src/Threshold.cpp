#include "Threshold.h"

void Threshold::set(threshold_t threshold) {
  if (threshold.sensor >= NUM_SENSOR) return;
  if (threshold.threshold >= NUM_THRESHOLD) return;

  thresholds.sensor[threshold.sensor].threshold[threshold.threshold] = threshold;
}

int Threshold::check(uint8_t sensor, uint8_t threshold, int range) {
  // range is the distance just read from sensor.
  // threshold is the threshold being checked.
  // Returns the index of the event to send or -1 if there is no event to be sent.
  // Gets the current state for this threshold for this sensor and checks
  // to see if the range has passed the near or far thresholds.

  State currentState = thresholds.sensor[sensor].threshold[threshold].currentState;

  if (currentState == State::Far) {
    // Send near event if range < near value.
    if (range < thresholds.sensor[sensor].threshold[threshold].valueNear) {
      thresholds.sensor[sensor].threshold[threshold].currentState = State::Near;
      return thresholds.sensor[sensor].threshold[threshold].eventIndexNear;
    }
  } else if (currentState == State::Near) {
    // Send far event if range > far value.
    if (range > thresholds.sensor[sensor].threshold[threshold].valueFar) {
      thresholds.sensor[sensor].threshold[threshold].currentState = State::Far;
      return thresholds.sensor[sensor].threshold[threshold].eventIndexFar;
    }
  } else {
    // Error in currentState.
    return -1;
  }

  // No change to this threshold.
  return -1;
}

State Threshold::getStateForEventIndex(uint16_t index) {

  // Find the threshold for this event index.
  for (uint8_t i = 0; i < NUM_SENSOR; i++) {
    for (uint8_t j = 0; j < NUM_THRESHOLD; j++) {
      if ((thresholds.sensor[i].threshold[j].eventIndexNear == index)
        || (thresholds.sensor[i].threshold[j].eventIndexFar == index)) {

        // Found the threshold, so return its current state.
        return thresholds.sensor[i].threshold[j].currentState;
      }
    }
  }

  // Threshold not found.
  return State::Unknown;
}

void Threshold::print() {
for (uint8_t i = 0; i < NUM_SENSOR; i++) {
  for (uint8_t j = 0; j < NUM_THRESHOLD; j++) {
    Serial.printf("\nSensor %d, threshold %d, near %d, far %d eventIndexNear=%d, eventIndexFar=%d",
                i,
                j,
                thresholds.sensor[i].threshold[j].valueNear,
                thresholds.sensor[i].threshold[j].valueFar,
                thresholds.sensor[i].threshold[j].eventIndexNear,
                thresholds.sensor[i].threshold[j].eventIndexFar
                );
    }
  }
}
