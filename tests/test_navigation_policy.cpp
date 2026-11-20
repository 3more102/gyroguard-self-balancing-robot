#include "../firmware/GyroGuard/navigation_policy.h"
#include <cassert>

int main() {
  // A valid open side is selectable even if the other side is blocked.
  assert(chooseAvoidanceTurn(true, 700, true, 250, 450) == AvoidanceTurn::LEFT);
  assert(chooseAvoidanceTurn(true, 250, true, 700, 450) == AvoidanceTurn::RIGHT);

  // A missing echo is unknown, not an infinitely clear path.
  assert(chooseAvoidanceTurn(false, 2500, true, 700, 450) == AvoidanceTurn::RIGHT);
  assert(chooseAvoidanceTurn(true, 700, false, 2500, 450) == AvoidanceTurn::LEFT);
  assert(chooseAvoidanceTurn(false, 2500, false, 2500, 450) == AvoidanceTurn::HOLD);

  // Neither side is safe when both are blocked or only at the threshold.
  assert(chooseAvoidanceTurn(true, 300, true, 400, 450) == AvoidanceTurn::HOLD);
  assert(chooseAvoidanceTurn(true, 450, false, 0, 450) == AvoidanceTurn::HOLD);

  // When both sides are clear, choose the side with greater measured clearance.
  assert(chooseAvoidanceTurn(true, 600, true, 900, 450) == AvoidanceTurn::RIGHT);
  assert(chooseAvoidanceTurn(true, 900, true, 600, 450) == AvoidanceTurn::LEFT);

  // Equal clearances use a deterministic left-turn tie break.
  assert(chooseAvoidanceTurn(true, 800, true, 800, 450) == AvoidanceTurn::LEFT);
  return 0;
}
