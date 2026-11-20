#pragma once

#include <math.h>

// Numerically stable online statistics (Welford's algorithm).
class RunningStats {
public:
  RunningStats() : count_(0), mean_(0.0f), m2_(0.0f) {}

  void add(float value) {
    ++count_;
    const float delta = value - mean_;
    mean_ += delta / static_cast<float>(count_);
    const float deltaAfterUpdate = value - mean_;
    m2_ += delta * deltaAfterUpdate;
  }

  unsigned long count() const { return count_; }
  float mean() const { return mean_; }

  float variance() const {
    if (count_ < 2) return 0.0f;
    const float value = m2_ / static_cast<float>(count_ - 1);
    return value > 0.0f ? value : 0.0f;
  }

  float standardDeviation() const {
    return sqrtf(variance());
  }

private:
  unsigned long count_;
  float mean_;
  float m2_;
};

// Reject startup calibration if the chassis moved, vibrated heavily, or the
// accelerometer magnitude is inconsistent with a stationary 1 g measurement.
inline bool isStationaryCalibration(
    const RunningStats &pitchAngle,
    const RunningStats &gyroRate,
    const RunningStats &accelMagnitudeG,
    float maxAbsPitchMeanDeg,
    float maxPitchStdDeg,
    float maxGyroStdDps,
    float minAccelNormG,
    float maxAccelNormG,
    float maxAccelNormStdG) {
  if (pitchAngle.count() < 2 || gyroRate.count() < 2 ||
      accelMagnitudeG.count() < 2) {
    return false;
  }

  return fabsf(pitchAngle.mean()) <= maxAbsPitchMeanDeg &&
         pitchAngle.standardDeviation() <= maxPitchStdDeg &&
         gyroRate.standardDeviation() <= maxGyroStdDps &&
         accelMagnitudeG.mean() >= minAccelNormG &&
         accelMagnitudeG.mean() <= maxAccelNormG &&
         accelMagnitudeG.standardDeviation() <= maxAccelNormStdG;
}
