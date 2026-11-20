#include "../firmware/GyroGuard/reset_policy.h"
#include <cassert>

int main() {
  using namespace GyroGuardReset;

  assert(isValidArmedMarker(kArmedMarkerMagic, kArmedMarkerInverse));
  assert(!isValidArmedMarker(0U, 0U));
  assert(!isValidArmedMarker(kArmedMarkerMagic, 0U));
  assert(!isValidArmedMarker(0U, kArmedMarkerInverse));

  assert(isApplicationWatchdogReset(kWatchdogResetFlag));
  assert(!isApplicationWatchdogReset(
      kWatchdogResetFlag | kExternalResetFlag));
  assert(isBrownOutReset(kBrownOutResetFlag));
  assert(!isBrownOutReset(kExternalResetFlag));

  assert(requiresInspection(0U, true));
  assert(requiresInspection(kWatchdogResetFlag, false));
  assert(requiresInspection(kBrownOutResetFlag, false));
  assert(!requiresInspection(0U, false));
  assert(!requiresInspection(kExternalResetFlag, false));

  // A bootloader upload timeout is an external-reset/watchdog combination,
  // not an application watchdog failure.
  assert(!requiresInspection(
      kWatchdogResetFlag | kExternalResetFlag, false));
  return 0;
}
