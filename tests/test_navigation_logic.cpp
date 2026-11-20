#include "../firmware/GyroGuard/navigation_logic.h"
#include <cassert>

int main() {
  // Forward motion requires a fresh, valid measurement at or beyond stop range.
  assert(!frontPathClear(false, 1000, 320));
  assert(!frontPathClear(true, 319, 320));
  assert(frontPathClear(true, 320, 320));
  assert(frontPathClear(true, 1200, 320));

  // Unknown readings and two blocked sides must never produce a turn command.
  ScanDecision decision = chooseScanDirection(false, 900, false, 900, 450);
  assert(decision.hold);
  assert(decision.turnDirection == 0.0f);

  decision = chooseScanDirection(true, 900, false, 900, 450);
  assert(decision.hold);
  assert(decision.turnDirection == 0.0f);

  decision = chooseScanDirection(true, 300, true, 400, 450);
  assert(decision.hold);
  assert(decision.turnDirection == 0.0f);

  // Greater clearance on the left means turn left (-1).
  decision = chooseScanDirection(true, 800, true, 500, 450);
  assert(!decision.hold);
  assert(decision.turnDirection == -1.0f);

  // Greater clearance on the right means turn right (+1).
  decision = chooseScanDirection(true, 500, true, 800, 450);
  assert(!decision.hold);
  assert(decision.turnDirection == 1.0f);

  // One side is sufficient only when both sensors are valid; a valid open
  // direction is preferred even if the other side is closer to an obstacle.
  decision = chooseScanDirection(true, 500, true, 300, 450);
  assert(!decision.hold);
  assert(decision.turnDirection == -1.0f);

  decision = chooseScanDirection(true, 300, true, 500, 450);
  assert(!decision.hold);
  assert(decision.turnDirection == 1.0f);

  return 0;
}
