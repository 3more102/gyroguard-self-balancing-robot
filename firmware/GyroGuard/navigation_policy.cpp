#include "navigation_policy.h"

AvoidanceTurn chooseAvoidanceTurn(bool leftValid, uint16_t leftDistanceMm,
                                  bool rightValid, uint16_t rightDistanceMm,
                                  uint16_t clearThresholdMm) {
  const bool leftClear = leftValid && leftDistanceMm > clearThresholdMm;
  const bool rightClear = rightValid && rightDistanceMm > clearThresholdMm;

  if (leftClear && rightClear) {
    return (leftDistanceMm >= rightDistanceMm)
        ? AvoidanceTurn::LEFT : AvoidanceTurn::RIGHT;
  }
  if (leftClear) return AvoidanceTurn::LEFT;
  if (rightClear) return AvoidanceTurn::RIGHT;
  return AvoidanceTurn::HOLD;
}
