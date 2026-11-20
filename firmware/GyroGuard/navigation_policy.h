#pragma once

#include <stdint.h>

enum class AvoidanceAction : uint8_t {
  TURN_LEFT,
  TURN_RIGHT,
  REVERSE,
  HOLD
};

struct RangeObservation {
  uint16_t distanceMm;
  bool valid;
};

// Invalid measurements are unknown, never evidence of a clear path.
inline AvoidanceAction selectAvoidanceAction(
    RangeObservation left, RangeObservation right,
    uint16_t clearDistanceMm) {
  const bool leftClear = left.valid && left.distanceMm >= clearDistanceMm;
  const bool rightClear = right.valid && right.distanceMm >= clearDistanceMm;

  if (leftClear && rightClear) {
    return (left.distanceMm >= right.distanceMm)
        ? AvoidanceAction::TURN_LEFT
        : AvoidanceAction::TURN_RIGHT;
  }
  if (leftClear) return AvoidanceAction::TURN_LEFT;
  if (rightClear) return AvoidanceAction::TURN_RIGHT;

  if (left.valid && right.valid) return AvoidanceAction::REVERSE;
  return AvoidanceAction::HOLD;
}
