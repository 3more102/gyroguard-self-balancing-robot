#pragma once

#include <stdint.h>

enum class ScanAction : uint8_t {
  Hold,
  Turn,
  Reverse
};

struct ScanDecision {
  ScanAction action;
  int8_t turnDirection; // -1 = left, +1 = right, 0 = hold
};

// A missing echo is unknown, not clear. Unknown sides contribute zero
// clearance; if both sides are unknown, the only safe decision is to hold.
inline ScanDecision decideScan(bool leftValid, uint16_t leftDistanceMm,
                               bool rightValid, uint16_t rightDistanceMm,
                               uint16_t minimumClearanceMm) {
  if (!leftValid && !rightValid) {
    return {ScanAction::Hold, 0};
  }

  const uint16_t left = leftValid ? leftDistanceMm : 0U;
  const uint16_t right = rightValid ? rightDistanceMm : 0U;
  const int8_t direction = (left > right) ? -1 : 1;
  const bool bothSidesTight =
      left < minimumClearanceMm && right < minimumClearanceMm;

  return {bothSidesTight ? ScanAction::Reverse : ScanAction::Turn,
          direction};
}
