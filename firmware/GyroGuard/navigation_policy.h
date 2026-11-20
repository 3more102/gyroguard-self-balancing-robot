#pragma once

#include <stdint.h>

namespace NavigationPolicy {

inline bool frontRangeAllowsTravel(bool valid, uint16_t distanceMm,
                                   uint16_t stopDistanceMm) {
  return valid && distanceMm >= stopDistanceMm;
}

// Direction convention: -1 = left, +1 = right, 0 = no trustworthy path.
inline int8_t chooseTurnDirection(bool leftValid, uint16_t leftDistanceMm,
                                   bool rightValid, uint16_t rightDistanceMm) {
  if (!leftValid && !rightValid) return 0;
  if (!leftValid) return 1;
  if (!rightValid) return -1;
  return (leftDistanceMm > rightDistanceMm) ? -1 : 1;
}

}  // namespace NavigationPolicy
