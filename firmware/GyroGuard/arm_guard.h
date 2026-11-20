#pragma once
#include <math.h>
#include <stdint.h>

// Unsigned subtraction keeps this age check correct across micros() rollover.
// A timestamp in the future produces a large age and is rejected.
inline bool isPitchSampleFresh(uint32_t sampleTimeUs, uint32_t nowUs,
                               uint32_t maxAgeUs) {
  return static_cast<uint32_t>(nowUs - sampleTimeUs) <= maxAgeUs;
}

// A recent IMU sample, near-upright chassis, and low pitch rate are required
// before motor control can be armed.
inline bool canArmBalance(bool faultLatched, bool pitchValid, float pitchDeg,
                          float pitchRateDps, float maxAbsPitchDeg,
                          float maxAbsPitchRateDps, uint32_t sampleTimeUs,
                          uint32_t nowUs, uint32_t maxSampleAgeUs) {
  if (faultLatched || !pitchValid ||
      !isPitchSampleFresh(sampleTimeUs, nowUs, maxSampleAgeUs) ||
      !isfinite(pitchDeg) || !isfinite(pitchRateDps) ||
      !isfinite(maxAbsPitchDeg) || !isfinite(maxAbsPitchRateDps) ||
      maxAbsPitchDeg < 0.0f || maxAbsPitchRateDps < 0.0f) return false;
  return fabsf(pitchDeg) <= maxAbsPitchDeg &&
         fabsf(pitchRateDps) <= maxAbsPitchRateDps;
}
