#include "../firmware/GyroGuard/imu_calibration_stats.h"
#include <cassert>
#include <cmath>

static bool near(float a, float b, float epsilon = 1.0e-4f) {
  return std::fabs(a - b) <= epsilon;
}

int main() {
  float zeroAngle = 0.0f;
  float gyroBias = 0.0f;

  ImuCalibrationStats stationary;
  for (uint16_t i = 0; i < 500; ++i) {
    stationary.addSample(3.25f, 0.40f, 1.0f, 0.70f);
  }
  assert(stationary.finish(zeroAngle, gyroBias));
  assert(near(zeroAngle, 3.25f));
  assert(near(gyroBias, 0.40f));

  ImuCalibrationStats movingPitch;
  for (uint16_t i = 0; i < 500; ++i) {
    movingPitch.addSample((i & 1U) ? 4.0f : -4.0f, 0.2f, 1.0f, 0.5f);
  }
  assert(!movingPitch.finish(zeroAngle, gyroBias));

  ImuCalibrationStats movingGyro;
  for (uint16_t i = 0; i < 500; ++i) {
    movingGyro.addSample(2.0f, (i & 1U) ? 4.0f : -4.0f, 1.0f, 4.1f);
  }
  assert(!movingGyro.finish(zeroAngle, gyroBias));

  ImuCalibrationStats badGravityMagnitude;
  for (uint16_t i = 0; i < 500; ++i) {
    badGravityMagnitude.addSample(2.0f, 0.2f, 0.5f, 0.5f);
  }
  assert(!badGravityMagnitude.finish(zeroAngle, gyroBias));

  ImuCalibrationStats vibration;
  for (uint16_t i = 0; i < 500; ++i) {
    vibration.addSample(2.0f, 0.2f, (i & 1U) ? 1.2f : 0.8f, 0.5f);
  }
  assert(!vibration.finish(zeroAngle, gyroBias));

  ImuCalibrationStats insufficientSamples;
  for (uint16_t i = 0; i < 99; ++i) {
    insufficientSamples.addSample(2.0f, 0.2f, 1.0f, 0.5f);
  }
  assert(!insufficientSamples.finish(zeroAngle, gyroBias));

  return 0;
}
