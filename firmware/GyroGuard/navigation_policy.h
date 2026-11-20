#pragma once

#include <stdint.h>

// turnDirection: -1 = left, +1 = right, 0 = no verified safe direction.
struct NavigationDecision {
  int8_t turnDirection;
  bool reverseFirst;
  bool valid;
};

// Unknown side readings are never interpreted as clear space. A single
// measured side is usable only when it independently meets the clearance
// threshold. If both sides are measured but blocked, reverse before turning
// toward the side with the greater measured clearance.
inline NavigationDecision chooseNavigationDecision(
    bool leftValid, uint16_t leftMm, bool rightValid, uint16_t rightMm,
    uint16_t clearThresholdMm) {
  if (!leftValid && !rightValid) return {0, false, false};

  if (leftValid && !rightValid) {
    return leftMm >= clearThresholdMm
        ? NavigationDecision{-1, false, true}
        : NavigationDecision{0, false, false};
  }

  if (!leftValid && rightValid) {
    return rightMm >= clearThresholdMm
        ? NavigationDecision{1, false, true}
        : NavigationDecision{0, false, false};
  }

  const bool bothBlocked = leftMm < clearThresholdMm &&
                           rightMm < clearThresholdMm;
  const int8_t direction = leftMm > rightMm ? -1 : 1;
  return {direction, bothBlocked, true};
}
