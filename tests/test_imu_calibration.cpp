#include "../firmware/GyroGuard/imu_calibration.h"
#include <cassert>
#include <cmath>
#include <cstdint>

static bool near(float a, float b, float eps = 0.05f) {
  return std::fabs(a - b) <= eps;
}

static void addStationary(ImuCalibrationAccumulator &stats,
                          int16_t ax = 0, int16_t ay = 0,
                          int16_t az = 16384, int16_t gy = 262) {
  for (uint16_t i = 0; i < 500; ++i) {
    stats.add(ax, ay, az, 0, gy, 0);
  }
}

int main() {
  {
    ImuCalibrationAccumulator stats;
    float zeroAngle = 0.0f;
    float gyroBias = 0.0f;
    addStationary(stats);
    assert(stats.finish(500, zeroAngle, gyroBias));
    assert(near(zeroAngle, 0.0f));
    assert(near(gyroBias, 2.0f));
  }

  {
    ImuCalibrationAccumulator stats;
    float zeroAngle = 0.0f;
    float gyroBias = 0.0f;
    // A stable 10-degree mounting offset is valid and becomes the zero angle.
    addStationary(stats, -2845, 0, 16135, 0);
    assert(stats.finish(500, zeroAngle, gyroBias));
    assert(near(zeroAngle, 10.0f));
    assert(near(gyroBias, 0.0f));
  }

  {
    ImuCalibrationAccumulator stats;
    float zeroAngle = 0.0f;
    float gyroBias = 0.0f;
    // Alternating pitch motion and gyro rate must be rejected.
    for (uint16_t i = 0; i < 500; ++i) {
      const int16_t ax = (i & 1U) ? 2845 : -2845;
      const int16_t gy = (i & 1U) ? -2620 : 2620;
      stats.add(ax, 0, 16135, 0, gy, 0);
    }
    assert(!stats.finish(500, zeroAngle, gyroBias));
  }

  {
    ImuCalibrationAccumulator stats;
    float zeroAngle = 0.0f;
    float gyroBias = 0.0f;
    // A stationary-looking but implausible acceleration magnitude is rejected.
    addStationary(stats, 0, 0, 12000, 0);
    assert(!stats.finish(500, zeroAngle, gyroBias));
  }

  {
    ImuCalibrationAccumulator stats;
    float zeroAngle = 0.0f;
    float gyroBias = 0.0f;
    for (uint16_t i = 0; i < 99; ++i) stats.add(0, 0, 16384, 0, 0, 0);
    assert(!stats.finish(100, zeroAngle, gyroBias));
  }

  return 0;
}
