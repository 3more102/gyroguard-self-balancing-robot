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

  // A reading at the age limit is accepted; one millisecond older is stale.
  assert(gyroguardFrontRangeAllowsCruise(true, 800, 350, 100, 250, 320));
  assert(!gyroguardFrontRangeAllowsCruise(true, 800, 351, 100, 250, 320));

  // Unsigned age arithmetic remains correct when millis() wraps around.
  assert(gyroguardFrontRangeAllowsCruise(
      true, 800, 20U, 0xFFFFFFF0UL, 250U, 320U));

  // Both invalid side scans are unknown: no direction may be selected.
  GyroGuardScanDecision decision =
      gyroguardChooseScanDirection(false, 2500, false, 2500, 450);
  assert(!decision.usable);
  assert(!decision.reverseFirst);
  assert(decision.turnDirection == 0);

  // With only one valid side, choose only that known direction.
  decision = gyroguardChooseScanDirection(false, 2500, true, 600, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == 1);

  decision = gyroguardChooseScanDirection(true, 600, false, 2500, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == -1);

  // With two valid readings, choose the side with greater clearance.
  decision = gyroguardChooseScanDirection(true, 700, true, 500, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == -1);

  decision = gyroguardChooseScanDirection(true, 500, true, 700, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == 1);

  // If the best known side is too close, reverse before turning.
  decision = gyroguardChooseScanDirection(true, 400, true, 300, 450);
  assert(decision.usable && decision.reverseFirst);
  assert(decision.turnDirection == -1);

  // Exactly at the clear threshold does not require a reverse maneuver.
  decision = gyroguardChooseScanDirection(true, 450, true, 300, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == -1);

  // Ties choose right deterministically.
  decision = gyroguardChooseScanDirection(true, 500, true, 500, 450);
  assert(decision.usable && !decision.reverseFirst);
  assert(decision.turnDirection == 1);

  return 0;
}
