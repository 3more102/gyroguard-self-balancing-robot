#pragma once

#include <Arduino.h>

class MotorDriver {
public:
  void begin();
  void setSpeeds(float leftPwm, float rightPwm);
  void disable();

private:
  void setOne(uint8_t pwmPin, uint8_t in1, uint8_t in2, float command,
              int8_t motorSign);
};
