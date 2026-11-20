#pragma once

#include <math.h>
#include <stdint.h>

struct ImuCalibrationResult {
  uint16_t samples;
  float meanAngleDeg;
  float meanGyroDps;
  float meanAccelMagnitudeG;
  float angleStdDevDeg;
  float gyroStdDevDps;
};

class ImuCalibrationAccumulator {
public:
  ImuCalibrationAccumulator()
      : count_(0), angleMean_(0.0f), angleM2_(0.0f), gyroMean_(0.0f),
        gyroM2_(0.0f), accelMagnitudeMean_(0.0f) {}

  void add(float accelAngleDeg, float gyroRateDps, float accelMagnitudeG) {
    if (count_ == UINT16_MAX) return;
    ++count_;
    const float count = static_cast<float>(count_);

    const float angleDelta = accelAngleDeg - angleMean_;
    angleMean_ += angleDelta / count;
    angleM2_ += angleDelta * (accelAngleDeg - angleMean_);

    const float gyroDelta = gyroRateDps - gyroMean_;
    gyroMean_ += gyroDelta / count;
    gyroM2_ += gyroDelta * (gyroRateDps - gyroMean_);

    accelMagnitudeMean_ += (accelMagnitudeG - accelMagnitudeMean_) / count;
  }

  bool evaluate(float maxAngleStdDevDeg, float maxGyroStdDevDps,
                float minAccelMagnitudeG, float maxAccelMagnitudeG,
                ImuCalibrationResult &result) const {
    if (count_ < 2) return false;

    const float denominator = static_cast<float>(count_ - 1);
    result.samples = count_;
    result.meanAngleDeg = angleMean_;
    result.meanGyroDps = gyroMean_;
    result.meanAccelMagnitudeG = accelMagnitudeMean_;
    result.angleStdDevDeg = sqrtf(angleM2_ / denominator);
    result.gyroStdDevDps = sqrtf(gyroM2_ / denominator);

    return result.angleStdDevDeg <= maxAngleStdDevDeg &&
           result.gyroStdDevDps <= maxGyroStdDevDps &&
           result.meanAccelMagnitudeG >= minAccelMagnitudeG &&
           result.meanAccelMagnitudeG <= maxAccelMagnitudeG;
  }

private:
  uint16_t count_;
  float angleMean_;
  float angleM2_;
  float gyroMean_;
  float gyroM2_;
  float accelMagnitudeMean_;
};
