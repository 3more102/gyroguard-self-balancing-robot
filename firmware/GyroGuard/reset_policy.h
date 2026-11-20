#pragma once

#include <stdint.h>

namespace GyroGuardReset {

constexpr uint8_t kExternalResetFlag = static_cast<uint8_t>(1U << 1);
constexpr uint8_t kBrownOutResetFlag = static_cast<uint8_t>(1U << 2);
constexpr uint8_t kWatchdogResetFlag = static_cast<uint8_t>(1U << 3);

// Two-word marker reduces the chance that random SRAM contents look like an
// armed-state record after power loss or an unrelated reset.
constexpr uint16_t kArmedMarkerMagic = 0xB17A;
constexpr uint16_t kArmedMarkerInverse =
    static_cast<uint16_t>(~kArmedMarkerMagic);

inline bool isValidArmedMarker(uint16_t magic, uint16_t inverse) {
  return magic == kArmedMarkerMagic && inverse == kArmedMarkerInverse;
}

inline bool isApplicationWatchdogReset(uint8_t resetFlags) {
  return (resetFlags & kWatchdogResetFlag) != 0 &&
         (resetFlags & kExternalResetFlag) == 0;
}

inline bool isBrownOutReset(uint8_t resetFlags) {
  return (resetFlags & kBrownOutResetFlag) != 0;
}

inline bool requiresInspection(uint8_t resetFlags, bool wasBalanceArmed) {
  return wasBalanceArmed || isApplicationWatchdogReset(resetFlags) ||
         isBrownOutReset(resetFlags);
}

}  // namespace GyroGuardReset
