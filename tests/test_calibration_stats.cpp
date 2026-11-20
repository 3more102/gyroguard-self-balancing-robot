#include "../firmware/GyroGuard/calibration_stats.h"
#include <assert.h>
#include <math.h>

int main() {
  RunningStats pitch;
  RunningStats gyro;
  RunningStats accelMagnitude;

  for (int i = 0; i < 500; ++i) {
    const float noise = static_cast<float>((i % 5) - 2) * 0.001f;
    pitch.add(0.25f + noise);
    gyro.add(-0.10f + noise);
    accelMagnitude.add(1.0f + noise * 0.1f);
  }

  assert(pitch.count() == 500UL);
  assert(fabsf(pitch.mean() - 0.25f) < 1.0e-4f);
  assert(isStationaryCalibration(pitch, gyro, accelMagnitude,
                                 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats movingPitch;
  RunningStats stableGyro;
  RunningStats stableAccel;
  for (int i = 0; i < 500; ++i) {
    movingPitch.add((i % 2 == 0) ? -3.0f : 3.0f);
    stableGyro.add(0.0f);
    stableAccel.add(1.0f);
  }
  assert(!isStationaryCalibration(movingPitch, stableGyro, stableAccel,
                                  1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats stablePitch;
  RunningStats stableGyro2;
  RunningStats lowAccel;
  for (int i = 0; i < 500; ++i) {
    stablePitch.add(0.0f);
    stableGyro2.add(0.0f);
    lowAccel.add(0.50f);
  }
  assert(!isStationaryCalibration(stablePitch, stableGyro2, lowAccel,
                                  1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats stablePitch2;
  RunningStats stableGyro3;
  RunningStats noisyAccel;
  for (int i = 0; i < 500; ++i) {
    stablePitch2.add(0.0f);
    stableGyro3.add(0.0f);
    noisyAccel.add((i % 2 == 0) ? 0.70f : 1.30f);
  }
  assert(!isStationaryCalibration(stablePitch2, stableGyro3, noisyAccel,
                                  1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats insufficient;
  insufficient.add(0.0f);
  assert(!isStationaryCalibration(insufficient, insufficient, insufficient,
                                  1.25f, 1.50f, 0.80f, 1.20f, 0.08f));
  return 0;
}
