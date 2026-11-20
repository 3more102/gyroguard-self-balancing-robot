#include "sonar.h"
#include "config.h"
#include <avr/interrupt.h>

volatile uint32_t Ultrasonic::echoRiseUs_ = 0;
volatile uint32_t Ultrasonic::echoPulseUs_ = 0;
volatile bool Ultrasonic::echoComplete_ = false;
volatile bool Ultrasonic::echoWaitingForRise_ = false;

void Ultrasonic::begin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);
  waiting_ = false;
  resultReady_ = false;
  valid_ = false;

  // Arduino Uno A1 = PCINT9 (pin-change interrupt group 1).
  noInterrupts();
  PCIFR |= _BV(PCIF1);
  PCMSK1 |= _BV(PCINT9);
  PCICR |= _BV(PCIE1);
  interrupts();
}

void Ultrasonic::onEchoChange() {
  const bool high = (PINC & _BV(PC1)) != 0;
  const uint32_t now = micros();
  if (high) {
    echoRiseUs_ = now;
    echoWaitingForRise_ = true;
  } else if (echoWaitingForRise_) {
    echoPulseUs_ = now - echoRiseUs_;
    echoWaitingForRise_ = false;
    echoComplete_ = true;
  }
}

ISR(PCINT1_vect) {
  Ultrasonic::onEchoChange();
}

bool Ultrasonic::startMeasurement(uint32_t nowUs) {
  if (waiting_) return false;
  noInterrupts();
  echoComplete_ = false;
  echoWaitingForRise_ = false;
  interrupts();

  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);

  triggerUs_ = nowUs;
  waiting_ = true;
  resultReady_ = false;
  return true;
}

void Ultrasonic::cancelMeasurement() {
  waiting_ = false;
  resultReady_ = false;
  valid_ = false;
  noInterrupts();
  echoComplete_ = false;
  echoWaitingForRise_ = false;
  interrupts();
}

void Ultrasonic::update(uint32_t nowUs) {
  if (!waiting_) return;

  bool complete;
  uint32_t pulse;
  noInterrupts();
  complete = echoComplete_;
  pulse = echoPulseUs_;
  if (complete) echoComplete_ = false;
  interrupts();

  if (complete) {
    // Sound speed is approximately 0.343 mm/us; divide by two for round trip.
    const uint32_t mm = (pulse * 343UL) / 2000UL;
    distanceMm_ = (mm > 65535UL) ? 65535U : static_cast<uint16_t>(mm);
    valid_ = distanceMm_ >= SONAR_MIN_RANGE_MM &&
             distanceMm_ <= SONAR_MAX_RANGE_MM;
    waiting_ = false;
    resultReady_ = true;
  } else if (static_cast<uint32_t>(nowUs - triggerUs_) > 30000UL) {
    waiting_ = false;
    valid_ = false;
    distanceMm_ = 0;
    resultReady_ = true;
  }
}

bool Ultrasonic::takeDistanceMm(uint16_t &distanceMm, bool &valid) {
  if (!resultReady_) return false;
  resultReady_ = false;
  valid = valid_;
  distanceMm = distanceMm_;
  return true;
}

bool Ultrasonic::busy() const {
  return waiting_;
}
