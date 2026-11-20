#include "../firmware/GyroGuard/imu_validation.h"
#include <cassert>
#include <cmath>

static bool near(float a, float b, float eps = 1.0e-4f) {
  return std::fabs(a - b) <= eps;
}

static bool validWindow(const RunningStats &angle,
                        const RunningStats &gyroY,
                        const RunningStats &gyroMagnitude,
                        const RunningStats &accelMagnitude) {
  return calibrationWindowIsStationary(
      angle, gyroY, gyroMagnitude, accelMagnitude,
      2.5f, 15.0f, 3.0f, 5.0f, 0.75f, 1.25f, 0.08f);
}

static void addStableSamples(RunningStats &angle, RunningStats &gyroY,
                             RunningStats &gyroMagnitude,
                             RunningStats &accelMagnitude) {
  for (uint16_t i = 0; i < 500U; ++i) {
    const float sign = (i & 1U) ? 1.0f : -1.0f;
    angle.add(1.2f + sign * 0.05f);
    gyroY.add(0.3f + sign * 0.02f);
    gyroMagnitude.add(0.5f + sign * 0.02f);
    accelMagnitude.add(1.0f + sign * 0.002f);
  }
}

int main() {
  assert(isMpu6050Identity(0x68U));
  assert(!isMpu6050Identity(0x69U)); // WHO_AM_I bit 0 is hard-coded to zero.
  assert(!isMpu6050Identity(0xE8U)); // Reserved bit 7 is also hard-coded to zero.
  assert(!isMpu6050Identity(0x00U));
  assert(!isMpu6050Identity(0x70U));

  RunningStats known;
  known.add(1.0f);
  known.add(2.0f);
  known.add(3.0f);
  assert(near(known.mean(), 2.0f));
  assert(near(known.standardDeviation(), 1.0f));

  RunningStats stableAngle, stableGyroY, stableGyroMagnitude, stableAccelMagnitude;
  addStableSamples(stableAngle, stableGyroY, stableGyroMagnitude,
                   stableAccelMagnitude);
  assert(validWindow(stableAngle, stableGyroY, stableGyroMagnitude,
                     stableAccelMagnitude));

  RunningStats movingAngle, movingGyroY, movingGyroMagnitude, movingAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    movingAngle.add((i & 1U) ? 8.0f : 0.0f);
    movingGyroY.add(0.3f);
    movingGyroMagnitude.add(0.5f);
    movingAccelMagnitude.add(1.0f);
  }
  assert(!validWindow(movingAngle, movingGyroY, movingGyroMagnitude,
                      movingAccelMagnitude));

  RunningStats vibratingAngle, vibratingGyroY, vibratingGyroMagnitude, vibratingAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    vibratingAngle.add(1.0f);
    vibratingGyroY.add((i & 1U) ? 4.0f : -4.0f);
    vibratingGyroMagnitude.add(4.0f);
    vibratingAccelMagnitude.add(1.0f);
  }
  assert(!validWindow(vibratingAngle, vibratingGyroY, vibratingGyroMagnitude,
                      vibratingAccelMagnitude));

  RunningStats badGravityAngle, badGravityGyroY, badGravityGyroMagnitude, badGravityAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    badGravityAngle.add(1.0f);
    badGravityGyroY.add(0.0f);
    badGravityGyroMagnitude.add(0.0f);
    badGravityAccelMagnitude.add(0.5f);
  }
  assert(!validWindow(badGravityAngle, badGravityGyroY, badGravityGyroMagnitude,
                      badGravityAccelMagnitude));

  RunningStats noisyGravityAngle, noisyGravityGyroY, noisyGravityGyroMagnitude, noisyGravityAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    noisyGravityAngle.add(1.0f);
    noisyGravityGyroY.add(0.0f);
    noisyGravityGyroMagnitude.add(0.0f);
    noisyGravityAccelMagnitude.add((i & 1U) ? 1.5f : 0.5f);
  }
  assert(!validWindow(noisyGravityAngle, noisyGravityGyroY,
                      noisyGravityGyroMagnitude, noisyGravityAccelMagnitude));

  // A stable but significantly tilted pose must not become the zero reference.
  RunningStats tiltedAngle, tiltedGyroY, tiltedGyroMagnitude, tiltedAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    tiltedAngle.add(25.0f);
    tiltedGyroY.add(0.0f);
    tiltedGyroMagnitude.add(0.0f);
    tiltedAccelMagnitude.add(1.0f);
  }
  assert(!validWindow(tiltedAngle, tiltedGyroY, tiltedGyroMagnitude,
                      tiltedAccelMagnitude));

  // Constant yaw or pitch rotation has low variance but is not stationary.
  RunningStats rotatingAngle, rotatingGyroY, rotatingGyroMagnitude, rotatingAccelMagnitude;
  for (uint16_t i = 0; i < 500U; ++i) {
    rotatingAngle.add(1.0f);
    rotatingGyroY.add(0.0f);  // Example: rotation about the yaw axis.
    rotatingGyroMagnitude.add(10.0f);
    rotatingAccelMagnitude.add(1.0f);
  }
  assert(!validWindow(rotatingAngle, rotatingGyroY, rotatingGyroMagnitude,
                      rotatingAccelMagnitude));

  RunningStats tooFewAngle, tooFewGyroY, tooFewGyroMagnitude, tooFewAccelMagnitude;
  for (uint16_t i = 0; i < 99U; ++i) {
    tooFewAngle.add(1.0f);
    tooFewGyroY.add(0.0f);
    tooFewGyroMagnitude.add(0.0f);
    tooFewAccelMagnitude.add(1.0f);
  }
  assert(!validWindow(tooFewAngle, tooFewGyroY, tooFewGyroMagnitude,
                      tooFewAccelMagnitude));

  return 0;
}
