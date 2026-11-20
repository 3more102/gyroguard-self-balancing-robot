#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

int main() {
  const NavigationDecision leftClear =
      chooseNavigationDecision(true, 900, true, 300, 450);
  assert(leftClear.valid && leftClear.turnDirection == -1 &&
         !leftClear.reverseFirst);

  const NavigationDecision rightClear =
      chooseNavigationDecision(true, 250, true, 800, 450);
  assert(rightClear.valid && rightClear.turnDirection == 1 &&
         !rightClear.reverseFirst);

  const NavigationDecision oneValidLeft =
      chooseNavigationDecision(true, 700, false, 0, 450);
  assert(oneValidLeft.valid && oneValidLeft.turnDirection == -1 &&
         !oneValidLeft.reverseFirst);

  const NavigationDecision oneValidRight =
      chooseNavigationDecision(false, 0, true, 650, 450);
  assert(oneValidRight.valid && oneValidRight.turnDirection == 1 &&
         !oneValidRight.reverseFirst);

  const NavigationDecision unknown =
      chooseNavigationDecision(false, 0, false, 0, 450);
  assert(!unknown.valid && unknown.turnDirection == 0);

  const NavigationDecision blockedUnknown =
      chooseNavigationDecision(true, 200, false, 0, 450);
  assert(!blockedUnknown.valid && blockedUnknown.turnDirection == 0);

  const NavigationDecision bothBlocked =
      chooseNavigationDecision(true, 220, true, 350, 450);
  assert(bothBlocked.valid && bothBlocked.turnDirection == 1 &&
         bothBlocked.reverseFirst);

  const NavigationDecision equalClearance =
      chooseNavigationDecision(true, 600, true, 600, 450);
  assert(equalClearance.valid && equalClearance.turnDirection == 1 &&
         !equalClearance.reverseFirst);

  return 0;
}
