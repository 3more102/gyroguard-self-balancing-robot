#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

int main() {
  const NavigationDecision leftClear =
      chooseNavigationDecision(true, 900, true, 300, 450);
  assert(leftClear.valid && leftClear.turnDirection == -1);

  const NavigationDecision rightClear =
      chooseNavigationDecision(true, 250, true, 800, 450);
  assert(rightClear.valid && rightClear.turnDirection == 1);

  const NavigationDecision oneValidLeft =
      chooseNavigationDecision(true, 700, false, 0, 450);
  assert(oneValidLeft.valid && oneValidLeft.turnDirection == -1);

  const NavigationDecision oneValidRight =
      chooseNavigationDecision(false, 0, true, 650, 450);
  assert(oneValidRight.valid && oneValidRight.turnDirection == 1);

  // Missing echoes are unknown, not clear paths.
  const NavigationDecision unknown =
      chooseNavigationDecision(false, 0, false, 0, 450);
  assert(!unknown.valid && unknown.turnDirection == 0);

  const NavigationDecision blockedUnknown =
      chooseNavigationDecision(true, 200, false, 0, 450);
  assert(!blockedUnknown.valid && blockedUnknown.turnDirection == 0);

  // Both measured directions are below the clearance threshold. The robot
  // must hold rather than reverse because there is no rear-facing sensor.
  const NavigationDecision bothBlocked =
      chooseNavigationDecision(true, 220, true, 350, 450);
  assert(!bothBlocked.valid && bothBlocked.turnDirection == 0);

  // A measurement exactly at the configured clearance threshold is usable.
  const NavigationDecision threshold =
      chooseNavigationDecision(true, 450, false, 0, 450);
  assert(threshold.valid && threshold.turnDirection == -1);

  const NavigationDecision equalClearance =
      chooseNavigationDecision(true, 600, true, 600, 450);
  assert(equalClearance.valid && equalClearance.turnDirection == 1);

  return 0;
}
