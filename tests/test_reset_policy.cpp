#include "../firmware/GyroGuard/reset_policy.h"
#include <cassert>

int main() {
  using namespace GyroGuardReset;

  assert(selectResetCause(kWatchdogResetFlag, 0) == kWatchdogResetFlag);
  assert(selectResetCause(kBrownOutResetFlag, kWatchdogResetFlag) ==
         kBrownOutResetFlag);
  assert(selectResetCause(kExternalResetFlag, kWatchdogResetFlag) ==
         kExternalResetFlag);
  assert(selectResetCause(0, kWatchdogResetFlag) == kWatchdogResetFlag);
  assert(selectResetCause(0, kBrownOutResetFlag) == kBrownOutResetFlag);
  assert(selectResetCause(0, 1U) == 0U);

  assert(isApplicationWatchdogReset(kWatchdogResetFlag));
  assert(requiresInspection(kWatchdogResetFlag));

  const uint8_t uploadTimeout = kWatchdogResetFlag | kExternalResetFlag;
  assert(!isApplicationWatchdogReset(uploadTimeout));
  assert(!requiresInspection(uploadTimeout));

  assert(isBrownOutReset(kBrownOutResetFlag));
  assert(requiresInspection(kBrownOutResetFlag | 1U));
  assert(!requiresInspection(1U));
  assert(!requiresInspection(kExternalResetFlag));
  return 0;
}
