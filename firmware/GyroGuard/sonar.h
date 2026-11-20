#pragma once

#include <Arduino.h>

class Ultrasonic {
public:
  void begin();
  void update(uint32_t nowUs);
  bool startMeasurement(uint32_t nowUs);
  // Returns true when a measurement completed; valid is false on timeout or
  // an out-of-range echo.
  bool takeDistanceMm(uint16_t &distanceMm, bool &valid);
  bool busy() const;

  // Called by the A1 pin-change ISR.
  static void onEchoChange();

private:
  static volatile uint32_t echoRiseUs_;
  static volatile uint32_t echoPulseUs_;
  static volatile bool echoComplete_;
  static volatile bool echoWaitingForRise_;

  bool waiting_;
  bool resultReady_;
  uint32_t triggerUs_;
  uint16_t distanceMm_;
  bool valid_;
};
