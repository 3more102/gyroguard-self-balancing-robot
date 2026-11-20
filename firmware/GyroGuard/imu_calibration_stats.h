#pragma once

#include <stdint.h>
#include <math.h>

// Online Welford statistics keep the calibration check numerically stable
// without storing hundreds of samples on the ATmega328P.
class ImuCalibrationStats {
public:
  ImuCalibrationStats()
      : count_(0), angleMean_(0.0f), angleM2_(0.0f),
        gyroYMean_(0.0f), gyroYM2_(0.0f),
        accelMeanG_(0.0f), accelM2G_(0.0f),
        gyroMagnitudeMeanDps_(0.0f), gyroMagnitudeM2Dps_(0.0f) {}

  void addSample(float accelAngleDeg, float gyroYRateDps,
                 float accelMagnitudeG, float gyroMagnitudeDps) {
    ++count_;
    update(accelAngleDeg, count_, angleMean_, angleM2_);
    update(gyroYRateDps, count_, gyroYMean_, gyroYM2_);
    update(accelMagnitudeG, count_, accelMeanG_, accelM2G_);
    update(gyroMagnitudeDps, count_, gyroMagnitudeMeanDps_,
           gyroMagnitudeM2Dps_);
  }

  bool finish(float &zeroAngleDeg, float &gyroBiasDps) const {
    if (count_ < 100U) return false;

    const float inverseCount = 1.0f / static_cast<float>(count_);
    const float angleStdDev = sqrtf(angleM2_ * inverseCount);
    const float gyroYStdDev = sqrtf(gyroYM2_ * inverseCount);
    const float accelStdDevG = sqrtf(accelM2G_ * inverseCount);
    const float gyroMagnitudeStdDevDps =
        sqrtf(gyroMagnitudeM2Dps_ * inverseCount);

    // Limits are intentionally generous for a stationary MPU-6050 breakout,
    // while rejecting obvious hand motion, vibration and non-gravitational load.
    if (angleStdDev > 1.5f || gyroYStdDev > 2.0f ||
        gyroMagnitudeStdDevDps > 2.0f ||
        accelMeanG_ < 0.75f || accelMeanG_ > 1.25f ||
        accelStdDevG > 0.10f) {
      return false;
    }

    zeroAngleDeg = angleMean_;
    gyroBiasDps = gyroYMean_;
    return true;
  }

private:
  static void update(float sample, uint16_t count,
                     float &mean, float &sumSquaredDelta) {
    const float delta = sample - mean;
    mean += delta / static_cast<float>(count);
    const float deltaAfterUpdate = sample - mean;
    sumSquaredDelta += delta * deltaAfterUpdate;
  }

  uint16_t count_;
  float angleMean_;
  float angleM2_;
  float gyroYMean_;
  float gyroYM2_;
  float accelMeanG_;
  float accelM2G_;
  float gyroMagnitudeMeanDps_;
  float gyroMagnitudeM2Dps_;
};
