#pragma once
#include <math.h>

// A live IMU sample, near-upright chassis, and low pitch rate are required
// before motor control can be armed.
inline bool canArmBalance(bool faultLatched, bool pitchValid, float pitchDeg,
                          float pitchRateDps, float maxAbsPitchDeg,
                          float maxAbsPitchRateDps) {
  if (faultLatched || !pitchValid || !isfinite(pitchDeg) ||
      !isfinite(pitchRateDps) || !isfinite(maxAbsPitchDeg) ||
      !isfinite(maxAbsPitchRateDps) || maxAbsPitchDeg < 0.0f ||
      maxAbsPitchRateDps < 0.0f) return false;
  return fabsf(pitchDeg) <= maxAbsPitchDeg &&
         fabsf(pitchRateDps) <= maxAbsPitchRateDps;
}
