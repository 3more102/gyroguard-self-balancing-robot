#pragma once

#include <Arduino.h>

class BatteryMonitor {
public:
  void begin();
  bool update(uint32_t nowMs);
  bool ready() const;
  bool lowBattery() const;
  bool voltageValid() const;
  bool travelInhibited() const;
  uint16_t millivolts() const;

private:
  uint32_t lastSampleMs_;
  uint16_t filteredMillivolts_;
  bool initialized_;
  bool sampleValid_;
  bool travelInhibited_;
};
