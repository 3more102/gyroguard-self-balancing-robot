#pragma once

#include <math.h>
#include <stdint.h>

class RunningStats {
public:
  RunningStats() : count_(0), mean_(0.0f), sumSquares_(0.0f) {}

  void add(float value) {
    ++count_;
    const float delta = value - mean_;
    mean_ += delta / static_cast<float>(count_);
    const float deltaAfter = value - mean_;
    sumSquares_ += delta * deltaAfter;
  }

  uint16_t count() const { return count_; }
  float mean() const { return mean_; }

  float standardDeviation() const {
    if (count_ < 2U) return 0.0f;
    return sqrtf(sumSquares_ / static_cast<float>(count_ - 1U));
  }

private:
  uint16_t count_;
  float mean_;
  float sumSquares_;
};

inline bool isMpu6050Identity(uint8_t whoAmI) {
  // WHO_AM_I is fixed at 0x68; AD0 changes the bus address, not this value.
  // Bits 7 and 0 are reserved and hard-coded to zero by the device.
  return whoAmI == 0x68U;
}

inline bool calibrationWindowIsStationary(
    const RunningStats &accelAngleDeg,
    const RunningStats &gyroRateDps,
    const RunningStats &gyroMagnitudeDps,
    const RunningStats &accelMagnitudeG,
    float maxAccelAngleStdDeg,
    float maxAbsAccelAngleDeg,
    float maxGyroRateStdDps,
    float maxGyroMagnitudeDps,
    float minGravityG,
    float maxGravityG,
    float maxGravityStdG) {
  const uint16_t samples = accelAngleDeg.count();
  if (samples < 100U ||
      gyroRateDps.count() != samples ||
      gyroMagnitudeDps.count() != samples ||
      accelMagnitudeG.count() != samples) {
    return false;
  }

  // Variance alone is insufficient: a stable but tilted or rotating robot
  // could otherwise be mistaken for the upright, stationary calibration pose.
  if (fabsf(accelAngleDeg.mean()) > maxAbsAccelAngleDeg ||
      accelAngleDeg.standardDeviation() > maxAccelAngleStdDeg ||
      gyroRateDps.standardDeviation() > maxGyroRateStdDps ||
      gyroMagnitudeDps.mean() > maxGyroMagnitudeDps ||
      gyroMagnitudeDps.standardDeviation() > maxGyroRateStdDps ||
      accelMagnitudeG.standardDeviation() > maxGravityStdG) {
    return false;
  }

  const float meanGravityG = accelMagnitudeG.mean();
  return meanGravityG >= minGravityG && meanGravityG <= maxGravityG;
}
