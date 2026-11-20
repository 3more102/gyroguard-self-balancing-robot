#pragma once

#include <stdint.h>
#include <math.h>

// Host-testable statistics for the MPU-6050 startup calibration window.
// Samples are collected at the configured 200 Hz sensor rate.
class ImuCalibrationAccumulator {
public:
  ImuCalibrationAccumulator()
      : count_(0),
        meanPitchDeg_(0.0f), m2PitchDeg_(0.0f),
        meanGyroXDps_(0.0f), m2GyroXDps_(0.0f),
        meanGyroYDps_(0.0f), m2GyroYDps_(0.0f),
        meanGyroZDps_(0.0f), m2GyroZDps_(0.0f),
        meanAccelMagnitudeG_(0.0f), m2AccelMagnitudeG_(0.0f) {}

  void add(int16_t ax, int16_t ay, int16_t az,
           int16_t gx, int16_t gy, int16_t gz) {
    const float axF = static_cast<float>(ax);
    const float ayF = static_cast<float>(ay);
    const float azF = static_cast<float>(az);
    const float pitchDeg = atan2f(-axF, azF) * 57.2957795f;
    const float accelMagnitudeG =
        sqrtf(axF * axF + ayF * ayF + azF * azF) / 16384.0f;

    ++count_;
    update(pitchDeg, count_, meanPitchDeg_, m2PitchDeg_);
    update(static_cast<float>(gx) / 131.0f, count_,
           meanGyroXDps_, m2GyroXDps_);
    update(static_cast<float>(gy) / 131.0f, count_,
           meanGyroYDps_, m2GyroYDps_);
    update(static_cast<float>(gz) / 131.0f, count_,
           meanGyroZDps_, m2GyroZDps_);
    update(accelMagnitudeG, count_,
           meanAccelMagnitudeG_, m2AccelMagnitudeG_);
  }

  // Reject a calibration window that contains obvious movement, vibration,
  // or an implausible accelerometer magnitude. The returned gyro bias is the
  // mean Y-axis rate; a constant sensor offset is expected and is calibrated.
  bool finish(uint16_t minimumSamples, float &zeroAngleDeg,
              float &gyroBiasDps) const {
    if (count_ < minimumSamples || count_ < 2) return false;

    const float pitchStdDev = standardDeviation(m2PitchDeg_, count_);
    const float gyroXStdDev = standardDeviation(m2GyroXDps_, count_);
    const float gyroYStdDev = standardDeviation(m2GyroYDps_, count_);
    const float gyroZStdDev = standardDeviation(m2GyroZDps_, count_);
    const float accelStdDev =
        standardDeviation(m2AccelMagnitudeG_, count_);

    if (meanAccelMagnitudeG_ < 0.85f ||
        meanAccelMagnitudeG_ > 1.15f ||
        accelStdDev > 0.06f ||
        pitchStdDev > 1.5f ||
        gyroXStdDev > 1.5f ||
        gyroYStdDev > 1.5f ||
        gyroZStdDev > 1.5f) {
      return false;
    }

    zeroAngleDeg = meanPitchDeg_;
    gyroBiasDps = meanGyroYDps_;
    return true;
  }

private:
  static void update(float sample, uint16_t count,
                     float &mean, float &sumSquaredError) {
    const float delta = sample - mean;
    mean += delta / static_cast<float>(count);
    const float deltaAfterUpdate = sample - mean;
    sumSquaredError += delta * deltaAfterUpdate;
  }

  static float standardDeviation(float sumSquaredError, uint16_t count) {
    const float variance =
        sumSquaredError / static_cast<float>(count - 1);
    return sqrtf(variance > 0.0f ? variance : 0.0f);
  }

  uint16_t count_;
  float meanPitchDeg_;
  float m2PitchDeg_;
  float meanGyroXDps_;
  float m2GyroXDps_;
  float meanGyroYDps_;
  float m2GyroYDps_;
  float meanGyroZDps_;
  float m2GyroZDps_;
  float meanAccelMagnitudeG_;
  float m2AccelMagnitudeG_;
};
