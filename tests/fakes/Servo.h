#pragma once
#include <Arduino.h>

extern int fakeServoAngle;

class Servo {
public:
  void attach(uint8_t) {}
  void write(uint8_t angle) { fakeServoAngle = angle; }
};
