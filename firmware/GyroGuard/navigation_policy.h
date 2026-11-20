#pragma once

#include <stdint.h>

namespace gyroguard {

struct RangeObservation {
  uint16_t distanceMm;
  bool valid;
};

// Returns -1 for left, +1 for right, or 0 when neither side is confirmed clear.
// A missing/invalid echo is unknown; it is never treated as free space.
inline int8_t chooseTurnDirection(const RangeObservation &left,
                                  const RangeObservation &right,
                                  uint16_t minimumClearanceMm) {
  const bool leftClear = left.valid && left.distanceMm >= minimumClearanceMm;
  const bool rightClear = right.valid && right.distanceMm >= minimumClearanceMm;

  if (leftClear && rightClear) {
    // Deterministic tie-break: turn right when both readings are equal.
    return (left.distanceMm > right.distanceMm) ? -1 : 1;
  }
  if (leftClear) return -1;
  if (rightClear) return 1;
  return 0;
}

}  // namespace gyroguard
