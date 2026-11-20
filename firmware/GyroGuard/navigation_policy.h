#pragma once

#include <stdint.h>

namespace GyroGuard {

enum class TurnDecision : int8_t {
  HOLD = 0,
  LEFT = -1,
  RIGHT = 1
};

// A missing or out-of-range echo is never evidence that the path is clear.
inline bool forwardPathIsClear(bool rangeValid, uint16_t rangeMm,
                               uint16_t stopDistanceMm) {
  return rangeValid && rangeMm >= stopDistanceMm;
}

// Invalid side measurements cannot be selected as a safe direction.
inline TurnDecision chooseTurnDirection(bool leftValid, uint16_t leftMm,
                                         bool rightValid, uint16_t rightMm,
                                         uint16_t minimumClearanceMm) {
  const bool leftClear = leftValid && leftMm >= minimumClearanceMm;
  const bool rightClear = rightValid && rightMm >= minimumClearanceMm;

  if (leftClear && rightClear) {
    return leftMm > rightMm ? TurnDecision::LEFT : TurnDecision::RIGHT;
  }
  if (leftClear) return TurnDecision::LEFT;
  if (rightClear) return TurnDecision::RIGHT;
  return TurnDecision::HOLD;
}

}  // namespace GyroGuard
