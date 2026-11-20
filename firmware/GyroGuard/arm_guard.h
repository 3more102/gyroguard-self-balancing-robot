#pragma once
#include <math.h>

// A live IMU sample, no latched fault, and an upright chassis are all
// required before motor control can be armed.
inline bool canArmBalance(bool faultLatched, bool pitchValid, float pitchDeg,
                          float maxAbsPitchDeg) {
  if (faultLatched || !pitchValid || !isfinite(pitchDeg) ||
      !isfinite(maxAbsPitchDeg) || maxAbsPitchDeg < 0.0f) return false;
  return fabsf(pitchDeg) <= maxAbsPitchDeg;
}
