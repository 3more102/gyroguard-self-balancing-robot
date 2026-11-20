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
                                 10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats movingPitch;
  RunningStats stableGyro;
  RunningStats stableAccel;
  for (int i = 0; i < 500; ++i) {
    movingPitch.add((i % 2 == 0) ? -3.0f : 3.0f);
    stableGyro.add(0.0f);
    stableAccel.add(1.0f);
  }
  assert(!isStationaryCalibration(movingPitch, stableGyro, stableAccel,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats stablePitch;
  RunningStats stableGyro2;
  RunningStats lowAccel;
  for (int i = 0; i < 500; ++i) {
    stablePitch.add(0.0f);
    stableGyro2.add(0.0f);
    lowAccel.add(0.50f);
  }
  assert(!isStationaryCalibration(stablePitch, stableGyro2, lowAccel,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats stablePitch2;
  RunningStats stableGyro3;
  RunningStats noisyAccel;
  for (int i = 0; i < 500; ++i) {
    stablePitch2.add(0.0f);
    stableGyro3.add(0.0f);
    noisyAccel.add((i % 2 == 0) ? 0.70f : 1.30f);
  }
  assert(!isStationaryCalibration(stablePitch2, stableGyro3, noisyAccel,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));


  RunningStats tiltedPitch;
  RunningStats stableGyro4;
  RunningStats stableAccel2;
  for (int i = 0; i < 500; ++i) {
    tiltedPitch.add(15.0f);
    stableGyro4.add(0.0f);
    stableAccel2.add(1.0f);
  }
  assert(!isStationaryCalibration(tiltedPitch, stableGyro4, stableAccel2,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats stablePitch3;
  RunningStats noisyGyro;
  RunningStats stableAccel3;
  for (int i = 0; i < 500; ++i) {
    stablePitch3.add(0.0f);
    noisyGyro.add((i % 2 == 0) ? -3.0f : 3.0f);
    stableAccel3.add(1.0f);
  }
  assert(!isStationaryCalibration(stablePitch3, noisyGyro, stableAccel3,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));

  RunningStats insufficient;
  insufficient.add(0.0f);
  assert(!isStationaryCalibration(insufficient, insufficient, insufficient,
                                  10.0f, 1.25f, 1.50f, 0.80f, 1.20f, 0.08f));
  return 0;
}
