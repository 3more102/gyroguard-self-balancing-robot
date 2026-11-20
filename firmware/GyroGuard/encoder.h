#pragma once

#include <Arduino.h>

struct EncoderSnapshot {
  long left;
  long right;
};

class EncoderPair {
public:
  void begin();
  EncoderSnapshot snapshot() const;

private:
  static void leftISR();
  static void rightISR();
  static volatile long leftCount_;
  static volatile long rightCount_;
};
