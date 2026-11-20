#include "../firmware/GyroGuard/imu_calibration.h"
#include <cassert>
#include <cmath>

static bool near(float a, float b, float eps = 1.0e-4f) {
  return std::fabs(a - b) <= eps;
}

int main() {
  ImuCalibrationResult result{};

  ImuCalibrationAccumulator stationary;
  for (uint16_t i = 0; i < 500; ++i) {
    stationary.add(2.5f, 0.8f, 1.0f);
  }
  assert(stationary.evaluate(1.5f, 2.0f, 0.75f, 1.25f, result));
  assert(result.samples == 500);
  assert(near(result.meanAngleDeg, 2.5f));
  assert(near(result.meanGyroDps, 0.8f));
  assert(near(result.meanAccelMagnitudeG, 1.0f));
  assert(near(result.angleStdDevDeg, 0.0f));
  assert(near(result.gyroStdDevDps, 0.0f));

  ImuCalibrationAccumulator moving;
  for (uint16_t i = 0; i < 100; ++i) {
    const float sign = (i & 1U) ? 1.0f : -1.0f;
    moving.add(sign * 3.0f, sign * 5.0f, 1.0f);
  }
  assert(!moving.evaluate(1.5f, 2.0f, 0.75f, 1.25f, result));

  ImuCalibrationAccumulator badGravity;
  for (uint16_t i = 0; i < 100; ++i) {
    badGravity.add(0.0f, 0.0f, 0.4f);
  }
  assert(!badGravity.evaluate(1.5f, 2.0f, 0.75f, 1.25f, result));

  ImuCalibrationAccumulator insufficient;
  insufficient.add(0.0f, 0.0f, 1.0f);
  assert(!insufficient.evaluate(1.5f, 2.0f, 0.75f, 1.25f, result));

  return 0;
}
