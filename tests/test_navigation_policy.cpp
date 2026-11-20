#include "../firmware/GyroGuard/navigation_policy.h"

#include <cassert>
#include <cstdint>

int main() {
  using NavigationPolicy::chooseTurnDirection;
  using NavigationPolicy::frontRangeAllowsTravel;

  assert(!frontRangeAllowsTravel(false, 2500, 320));
  assert(!frontRangeAllowsTravel(true, 0, 320));
  assert(!frontRangeAllowsTravel(true, 319, 320));
  assert(frontRangeAllowsTravel(true, 320, 320));
  assert(frontRangeAllowsTravel(true, 1200, 320));

  assert(chooseTurnDirection(true, 900, true, 500) == -1);
  assert(chooseTurnDirection(true, 400, true, 800) == 1);

  assert(chooseTurnDirection(false, 0, true, 700) == 1);
  assert(chooseTurnDirection(true, 700, false, 0) == -1);

  assert(chooseTurnDirection(false, 0, false, 0) == 0);

  return 0;
}
