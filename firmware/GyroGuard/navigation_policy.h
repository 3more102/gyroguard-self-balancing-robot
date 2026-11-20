#pragma once

#include <stdint.h>

// turnDirection: -1 = left, +1 = right, 0 = no verified safe direction.
struct NavigationDecision {
  int8_t turnDirection;
  bool valid;
};

// Unknown or blocked readings are never interpreted as free space. A turn is
// authorized only when at least one measured side meets the clearance limit.
// This robot has no rear sensor, so this policy never requests reverse motion.
inline NavigationDecision chooseNavigationDecision(
    bool leftValid, uint16_t leftMm, bool rightValid, uint16_t rightMm,
    uint16_t clearThresholdMm) {
  const bool leftClear = leftValid && leftMm >= clearThresholdMm;
  const bool rightClear = rightValid && rightMm >= clearThresholdMm;

  if (leftClear && rightClear) {
    // Deterministic tie-break: turn right when clearances are equal.
    return {leftMm > rightMm ? static_cast<int8_t>(-1)
                             : static_cast<int8_t>(1), true};
  }
  if (leftClear) return {-1, true};
  if (rightClear) return {1, true};
  return {0, false};
}
