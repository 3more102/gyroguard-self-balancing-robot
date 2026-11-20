#include "pid.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float outputMin, float outputMax)
    : kp_(kp), ki_(ki), kd_(kd), outputMin_(outputMin),
      outputMax_(outputMax), integral_(0.0f), previousError_(0.0f),
      hasPrevious_(false) {}

void PIDController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  hasPrevious_ = false;
}

float PIDController::update(float error, float dtSeconds,
                            float derivativeMeasurement,
                            bool useDerivativeMeasurement) {
  if (!(dtSeconds > 0.0f) || dtSeconds > 0.1f) return 0.0f;

  float derivative = 0.0f;
  if (useDerivativeMeasurement) {
    derivative = derivativeMeasurement;
  } else if (hasPrevious_) {
    derivative = (error - previousError_) / dtSeconds;
  }

  const float candidateIntegral = integral_ + error * dtSeconds;
  const float candidateOutput = kp_ * error + ki_ * candidateIntegral
                              + kd_ * derivative;
  const bool pushesHigh = candidateOutput > outputMax_ && error > 0.0f;
  const bool pushesLow = candidateOutput < outputMin_ && error < 0.0f;
  if (!pushesHigh && !pushesLow) integral_ = candidateIntegral;

  previousError_ = error;
  hasPrevious_ = true;

  const float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  if (output > outputMax_) return outputMax_;
  if (output < outputMin_) return outputMin_;
  return output;
}
