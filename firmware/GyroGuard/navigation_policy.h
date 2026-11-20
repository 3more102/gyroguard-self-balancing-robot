#pragma once

#include <stdint.h>

// turnDirection: -1 = left, +1 = right, 0 = no verified safe direction.
struct NavigationDecision {
  int8_t turnDirection;
  bool valid;
};

// Unknown side readings are never interpreted as clear space. A side is
// selectable only when its measured range meets the clearance threshold.
// If neither side is verified clear, hold; this robot has no rear sensor.
inline NavigationDecision chooseNavigationDecision(
    bool leftValid, uint16_t leftMm, bool rightValid, uint16_t rightMm,
    uint16_t clearThresholdMm) {
  const bool leftClear = leftValid && leftMm >= clearThresholdMm;
  const bool rightClear = rightValid && rightMm >= clearThresholdMm;

  if (!leftClear && !rightClear) return {0, false};
  if (leftClear && !rightClear) return {-1, true};
  if (!leftClear && rightClear) return {1, true};

  // Both sides are clear: select the side with greater measured clearance.
  // Equal readings use a deterministic right-turn tie break.
  return {leftMm > rightMm ? -1 : 1, true};
}
