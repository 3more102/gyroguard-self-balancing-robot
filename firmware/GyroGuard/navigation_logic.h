#pragma once

#include <stdint.h>

// turnDirection: -1 = turn left, +1 = turn right.
struct ScanDecision {
  bool hold;
  float turnDirection;
};

inline bool frontPathClear(bool rangeValid, uint16_t rangeMm,
                           uint16_t stopDistanceMm) {
  return rangeValid && rangeMm >= stopDistanceMm;
}

// A turn is selected only when both side measurements are valid and at least
// one side has the configured clearance. Unknown readings are never treated
// as open space.
inline ScanDecision chooseScanDirection(bool leftValid, uint16_t leftMm,
                                         bool rightValid, uint16_t rightMm,
                                         uint16_t clearDistanceMm) {
  if (!leftValid || !rightValid) return {true, 0.0f};

  if (leftMm < clearDistanceMm && rightMm < clearDistanceMm) {
    return {true, 0.0f};
  }

  // Positive motor differential (left faster than right) turns right.
  return {false, (leftMm > rightMm) ? -1.0f : 1.0f};
}
