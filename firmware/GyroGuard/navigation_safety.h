#pragma once

#include <stdint.h>

// Forward travel is allowed only with a valid, recent range beyond the stop
// threshold. Unsigned subtraction keeps the age check safe across millis() wrap.
inline bool gyroguardFrontRangeAllowsCruise(bool rangeValid,
                                            uint16_t distanceMm,
                                            uint32_t nowMs,
                                            uint32_t lastMeasurementMs,
                                            uint32_t maxAgeMs,
                                            uint16_t stopDistanceMm) {
  const uint32_t ageMs = nowMs - lastMeasurementMs;
  return rangeValid && ageMs <= maxAgeMs && distanceMm >= stopDistanceMm;
}

struct GyroGuardScanDecision {
  bool usable;
  bool reverseFirst;
  int8_t turnDirection;  // -1 = left, +1 = right, 0 = no safe decision
};

// Unknown side readings are never treated as open space. If both readings are
// invalid, the caller must not choose a timed turn.
inline GyroGuardScanDecision gyroguardChooseScanDirection(
    bool leftValid, uint16_t leftDistanceMm,
    bool rightValid, uint16_t rightDistanceMm,
    uint16_t clearDistanceMm) {
  if (!leftValid && !rightValid) {
    return {false, false, 0};
  }

  int8_t direction = 0;
  uint16_t bestDistanceMm = 0;

  if (leftValid && rightValid) {
    // On an exact tie, choose right deterministically.
    if (leftDistanceMm > rightDistanceMm) {
      direction = -1;
      bestDistanceMm = leftDistanceMm;
    } else {
      direction = 1;
      bestDistanceMm = rightDistanceMm;
    }
  } else if (leftValid) {
    direction = -1;
    bestDistanceMm = leftDistanceMm;
  } else {
    direction = 1;
    bestDistanceMm = rightDistanceMm;
  }

  return {true, bestDistanceMm < clearDistanceMm, direction};
}
