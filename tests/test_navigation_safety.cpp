#include "../firmware/GyroGuard/navigation_safety.h"
#include <cassert>
#include <cstdint>

int main() {
  // A disconnected sensor or timeout must never authorize forward travel.
  assert(!gyroguardFrontRangeAllowsCruise(false, 1000, 100, 100, 250, 320));

  // Fresh, clear space permits cruise.
  assert(gyroguardFrontRangeAllowsCruise(true, 500, 350, 100, 250, 320));

  // The configured stop boundary is inclusive; inside it, do not cruise.
  assert(!gyroguardFrontRangeAllowsCruise(true, 319, 350, 100, 250, 320));
  assert(gyroguardFrontRangeAllowsCruise(true, 320, 350, 100, 250, 320));

  // A stale reading blocks travel, including just beyond the timeout.
  assert(gyroguardFrontRangeAllowsCruise(true, 800, 350, 100, 250, 320));
  assert(!gyroguardFrontRangeAllowsCruise(true, 800, 351, 100, 250, 320));

  // Unsigned age arithmetic remains correct when millis() wraps around.
  assert(gyroguardFrontRangeAllowsCruise(
      true, 800, 20U, 0xFFFFFFF0UL, 250U, 320U));

  // Unknown scan directions are treated as blocked, not as maximum clearance.
  assert(gyroguardConservativeScanDistance(false, 0) == 0U);
  assert(gyroguardConservativeScanDistance(false, 2500) == 0U);
  assert(gyroguardConservativeScanDistance(true, 420) == 420U);

  return 0;
}
