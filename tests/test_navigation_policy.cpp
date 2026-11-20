#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

int main() {
  using gyroguard::RangeObservation;
  using gyroguard::chooseTurnDirection;

  // When both sides are clear, choose the side with more measured clearance.
  assert(chooseTurnDirection({900, true}, {600, true}, 450) == -1);
  assert(chooseTurnDirection({600, true}, {900, true}, 450) == 1);

  // A single confirmed-clear side is selectable; an invalid side is unknown.
  assert(chooseTurnDirection({700, true}, {0, false}, 450) == -1);
  assert(chooseTurnDirection({0, false}, {700, true}, 450) == 1);

  // An invalid reading must never be promoted to maximum clearance.
  assert(chooseTurnDirection({2500, false}, {400, true}, 450) == 0);
  assert(chooseTurnDirection({400, true}, {2500, false}, 450) == 0);

  // Neither side meeting the clearance threshold means no safe turn.
  assert(chooseTurnDirection({300, true}, {400, true}, 450) == 0);
  assert(chooseTurnDirection({0, false}, {0, false}, 450) == 0);

  // The configured threshold is inclusive; equal clearances use right tie-break.
  assert(chooseTurnDirection({450, true}, {450, true}, 450) == 1);

  return 0;
}
