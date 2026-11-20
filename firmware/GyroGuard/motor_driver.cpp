#include "motor_driver.h"
#include "config.h"
#include <math.h>

void MotorDriver::begin() {
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  disable();
}

void MotorDriver::setOne(uint8_t pwmPin, uint8_t in1, uint8_t in2,
                         float command, int8_t motorSign) {
  command *= static_cast<float>(motorSign);
  if (command > 255.0f) command = 255.0f;
  if (command < -255.0f) command = -255.0f;

  const int pwm = static_cast<int>(fabsf(command));
  if (command > 0.5f) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (command < -0.5f) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, pwm);
}

void MotorDriver::setSpeeds(float leftPwm, float rightPwm) {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  setOne(PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2, leftPwm, LEFT_MOTOR_SIGN);
  setOne(PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2, rightPwm, RIGHT_MOTOR_SIGN);
}

void MotorDriver::disable() {
  // STBY is active-low: shut down both H-bridges before changing other pins.
  digitalWrite(PIN_MOTOR_STBY, LOW);
  analogWrite(PIN_LEFT_PWM, 0);
  analogWrite(PIN_RIGHT_PWM, 0);
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}
