#pragma once

#include <stdint.h>

namespace GyroGuardReset {

constexpr uint8_t kExternalResetFlag = static_cast<uint8_t>(1U << 1);
constexpr uint8_t kBrownOutResetFlag = static_cast<uint8_t>(1U << 2);
constexpr uint8_t kWatchdogResetFlag = static_cast<uint8_t>(1U << 3);

inline uint8_t selectResetCause(uint8_t mcusrFlags,
                               uint8_t bootloaderFlags) {
  if (mcusrFlags != 0) return mcusrFlags;
  const uint8_t safetyFlags =
      bootloaderFlags & (kWatchdogResetFlag | kBrownOutResetFlag);
  return safetyFlags != 0 ? bootloaderFlags : 0;
}

inline bool isApplicationWatchdogReset(uint8_t resetFlags) {
  return (resetFlags & kWatchdogResetFlag) != 0 &&
         (resetFlags & kExternalResetFlag) == 0;
}

inline bool isBrownOutReset(uint8_t resetFlags) {
  return (resetFlags & kBrownOutResetFlag) != 0;
}

inline bool requiresInspection(uint8_t resetFlags) {
  return isApplicationWatchdogReset(resetFlags) ||
         isBrownOutReset(resetFlags);
}

}  // namespace GyroGuardReset
