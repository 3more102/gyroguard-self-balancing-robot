#include "encoder.h"
#include "config.h"

volatile long EncoderPair::leftCount_ = 0;
volatile long EncoderPair::rightCount_ = 0;

void EncoderPair::begin() {
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  leftCount_ = 0;
  rightCount_ = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightISR, CHANGE);
}

void EncoderPair::leftISR() {
  const bool a = digitalRead(PIN_LEFT_ENC_A);
  const bool b = digitalRead(PIN_LEFT_ENC_B);
  leftCount_ += ((a == b) ? 1L : -1L) * LEFT_ENCODER_SIGN;
}

void EncoderPair::rightISR() {
  const bool a = digitalRead(PIN_RIGHT_ENC_A);
  const bool b = digitalRead(PIN_RIGHT_ENC_B);
  rightCount_ += ((a == b) ? 1L : -1L) * RIGHT_ENCODER_SIGN;
}

EncoderSnapshot EncoderPair::snapshot() const {
  noInterrupts();
  EncoderSnapshot result{leftCount_, rightCount_};
  interrupts();
  return result;
}
