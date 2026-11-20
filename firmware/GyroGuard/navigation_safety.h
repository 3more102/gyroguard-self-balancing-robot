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

// A missing side-scan echo is unknown, not evidence of free space.
inline uint16_t gyroguardConservativeScanDistance(bool rangeValid,
                                                  uint16_t distanceMm) {
  return rangeValid ? distanceMm : 0U;
}
