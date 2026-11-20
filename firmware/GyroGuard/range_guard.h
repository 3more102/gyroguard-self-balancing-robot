#pragma once

#include <stdint.h>

// A stale or invalid front range must never authorize autonomous travel.
// Unsigned subtraction keeps the age calculation correct across millis() rollover.
inline bool isRangeSampleFresh(bool valid, uint32_t sampledAtMs,
                               uint32_t nowMs, uint32_t maxAgeMs) {
  return valid &&
         static_cast<uint32_t>(nowMs - sampledAtMs) <= maxAgeMs;
}
