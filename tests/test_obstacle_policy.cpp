#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

int main() {
  const uint16_t clearMm = 450;

  assert(selectAvoidanceAction({700, true}, {300, true}, clearMm) ==
         AvoidanceAction::TURN_LEFT);
  assert(selectAvoidanceAction({300, true}, {700, true}, clearMm) ==
         AvoidanceAction::TURN_RIGHT);

  // A single valid, clear side is preferred over an unknown side.
  assert(selectAvoidanceAction({0, false}, {650, true}, clearMm) ==
         AvoidanceAction::TURN_RIGHT);
  assert(selectAvoidanceAction({650, true}, {0, false}, clearMm) ==
         AvoidanceAction::TURN_LEFT);

  // Unknown readings are never interpreted as open space.
  assert(selectAvoidanceAction({0, false}, {0, false}, clearMm) ==
         AvoidanceAction::HOLD);
  assert(selectAvoidanceAction({320, true}, {0, false}, clearMm) ==
         AvoidanceAction::HOLD);

  // Reverse only when both sides returned valid but blocked readings.
  assert(selectAvoidanceAction({320, true}, {400, true}, clearMm) ==
         AvoidanceAction::REVERSE);

  // Exact clearance threshold is considered clear.
  assert(selectAvoidanceAction({450, true}, {450, true}, clearMm) ==
         AvoidanceAction::TURN_LEFT);

  return 0;
}
