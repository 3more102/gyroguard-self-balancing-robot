#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

using GyroGuard::TurnDecision;

int main() {
  // Forward motion requires a valid, sufficiently distant front measurement.
  assert(!GyroGuard::forwardPathIsClear(false, 2500, 320));
  assert(!GyroGuard::forwardPathIsClear(true, 319, 320));
  assert(GyroGuard::forwardPathIsClear(true, 320, 320));
  assert(GyroGuard::forwardPathIsClear(true, 1200, 320));

  // Select only a side with a valid measurement and adequate clearance.
  assert(GyroGuard::chooseTurnDirection(true, 900, true, 700, 450) ==
         TurnDecision::LEFT);
  assert(GyroGuard::chooseTurnDirection(true, 700, true, 900, 450) ==
         TurnDecision::RIGHT);
  assert(GyroGuard::chooseTurnDirection(true, 800, false, 2500, 450) ==
         TurnDecision::LEFT);
  assert(GyroGuard::chooseTurnDirection(false, 2500, true, 800, 450) ==
         TurnDecision::RIGHT);

  // No valid clear direction means hold, not a guessed turn.
  assert(GyroGuard::chooseTurnDirection(false, 2500, false, 2500, 450) ==
         TurnDecision::HOLD);
  assert(GyroGuard::chooseTurnDirection(true, 200, true, 300, 450) ==
         TurnDecision::HOLD);
  assert(GyroGuard::chooseTurnDirection(false, 0, true, 300, 450) ==
         TurnDecision::HOLD);

  return 0;
}
