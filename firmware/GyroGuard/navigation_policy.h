#pragma once

#include <stdint.h>

// turnDirection: -1 = left, +1 = right, 0 = no verified safe direction.
struct NavigationDecision {
  int8_t turnDirection;
  bool valid;
};

// Unknown side readings are never interpreted as clear space. A single
// measured side is usable only when it independently meets the clearance
// threshold. If both measured directions are blocked, no safe turn is returned.
inline NavigationDecision chooseNavigationDecision(
    bool leftValid, uint16_t leftMm, bool rightValid, uint16_t rightMm,
    uint16_t clearThresholdMm) {
  if (!leftValid && !rightValid) return {0, false};

  if (leftValid && !rightValid) {
    return leftMm >= clearThresholdMm
        ? NavigationDecision{-1, true}
        : NavigationDecision{0, false};
  }

  if (!leftValid && rightValid) {
    return rightMm >= clearThresholdMm
        ? NavigationDecision{1, true}
        : NavigationDecision{0, false};
  }

  if (leftMm < clearThresholdMm && rightMm < clearThresholdMm) {
    return {0, false};
  }

  const int8_t direction = leftMm > rightMm ? -1 : 1;
  return {direction, true};
}
