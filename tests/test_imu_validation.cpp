#include "../firmware/GyroGuard/imu_validation.h"
#include <cassert>
#include <cmath>

static bool near(float a, float b, float eps = 1.0e-4f) {
  return std::fabs(a - b) <= eps;
}

static bool validWindow(const RunningStats &angle,
                        const RunningStats &gyro,
                        const RunningStats &magnitude) {
  return calibrationWindowIsStationary(
      angle, gyro, magnitude, 2.5f, 3.0f, 0.75f, 1.25f, 0.08f);
}

static void addStableSamples(RunningStats &angle, RunningStats &gyro,
                             RunningStats &magnitude) {
  for (uint16_t i = 0; i < 500U; ++i) {
    const float sign = (i & 1U) ? 1.0f : -1.0f;
    angle.add(1.2f + sign * 0.05f);
    gyro.add(0.3f + sign * 0.02f);
    magnitude.add(1.0f + sign * 0.002f);
  }
}

int main() {
  assert(isMpu6050Identity(0x68U));
  assert(isMpu6050Identity(0x69U)); // Reserved bit 0 is ignored.
  assert(!isMpu6050Identity(0x00U));
  assert(!isMpu6050Identity(0x70U));

  RunningStats known;
  known.add(1.0f);
  known.add(2.0f);
  known.add(3.0f);
  assert(near(known.mean(), 2.0f));
  assert(near(known.standardDeviation(), 1.0f));

  RunningStats stableAngle, stableGyro, stableMagnitude;
  addStableSamples(stableAngle, stableGyro, stableMagnitude);
  assert(validWindow(stableAngle, stableGyro, stableMagnitude));

  RunningStats movingAngle, movingGyro, movingMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    movingAngle.add((i & 1U) ? 8.0f : 0.0f);
    movingGyro.add(0.3f);
    movingMagnitude.add(1.0f);
  }
  assert(!validWindow(movingAngle, movingGyro, movingMagnitude));

  RunningStats vibratingAngle, vibratingGyro, vibratingMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    vibratingAngle.add(1.0f);
    vibratingGyro.add((i & 1U) ? 4.0f : -4.0f);
    vibratingMagnitude.add(1.0f);
  }
  assert(!validWindow(vibratingAngle, vibratingGyro, vibratingMagnitude));

  RunningStats badGravityAngle, badGravityGyro, badGravityMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    badGravityAngle.add(1.0f);
    badGravityGyro.add(0.0f);
    badGravityMagnitude.add(0.5f);
  }
  assert(!validWindow(badGravityAngle, badGravityGyro, badGravityMagnitude));

  RunningStats noisyGravityAngle, noisyGravityGyro, noisyGravityMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    noisyGravityAngle.add(1.0f);
    noisyGravityGyro.add(0.0f);
    noisyGravityMagnitude.add((i & 1U) ? 1.5f : 0.5f);
  }
  assert(!validWindow(noisyGravityAngle, noisyGravityGyro,
                      noisyGravityMagnitude));

  RunningStats tooFewAngle, tooFewGyro, tooFewMagnitude;
  for (uint16_t i = 0; i < 99U; ++i) {
    tooFewAngle.add(1.0f);
    tooFewGyro.add(0.0f);
    tooFewMagnitude.add(1.0f);
  }
  assert(!validWindow(tooFewAngle, tooFewGyro, tooFewMagnitude));

  return 0;
}
