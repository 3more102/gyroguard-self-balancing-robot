#pragma once

#include <stdint.h>

// A direction is safe only when that side returned a valid range beyond the
// configured clearance threshold. HOLD means neither side is proven clear.
enum class AvoidanceTurn : int8_t {
  HOLD = 0,
  LEFT = -1,
  RIGHT = 1
};

AvoidanceTurn chooseAvoidanceTurn(bool leftValid, uint16_t leftDistanceMm,
                                  bool rightValid, uint16_t rightDistanceMm,
                                  uint16_t clearThresholdMm);
