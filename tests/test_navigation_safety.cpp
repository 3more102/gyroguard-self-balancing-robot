#include "../firmware/GyroGuard/navigation_safety.h"
#include <cassert>
#include <cstdint>

int main() {
  // Invalid or missing front range never authorizes autonomous cruise.
  assert(!gyroguardFrontRangeAllowsCruise(false, 1000, 100, 100, 250, 320));

  // Fresh clear space permits cruise; the stop boundary is inclusive.
  assert(gyroguardFrontRangeAllowsCruise(true, 500, 350, 100, 250, 320));
  assert(gyroguardFrontRangeAllowsCruise(true, 320, 350, 100, 250, 320));
  assert(!gyroguardFrontRangeAllowsCruise(true, 319, 350, 100, 250, 320));

  // Stale data blocks travel, including just beyond the timeout.
  assert(gyroguardFrontRangeAllowsCruise(true, 800, 350, 100, 250, 320));
  assert(!gyroguardFrontRangeAllowsCruise(true, 800, 351, 100, 250, 320));

  // Unsigned age arithmetic remains correct when millis() wraps around.
  assert(gyroguardFrontRangeAllowsCruise(
      true, 800, 20U, 0xFFFFFFF0UL, 250U, 320U));

  // Unknown scan directions are never interpreted as open space.
  assert(gyroguardChooseScanAction(false, 0, false, 0, 450) ==
         GyroGuardScanAction::Hold);
  assert(gyroguardChooseScanAction(false, 0, true, 700, 450) ==
         GyroGuardScanAction::TurnRight);
  assert(gyroguardChooseScanAction(true, 700, false, 0, 450) ==
         GyroGuardScanAction::TurnLeft);

  // One known but blocked side plus one unknown side is not a safe turn.
  assert(gyroguardChooseScanAction(true, 300, false, 0, 450) ==
         GyroGuardScanAction::Hold);
  assert(gyroguardChooseScanAction(false, 0, true, 300, 450) ==
         GyroGuardScanAction::Hold);

  // Two valid blocked sides trigger a bounded reverse-and-rescan decision.
  assert(gyroguardChooseScanAction(true, 300, true, 420, 450) ==
         GyroGuardScanAction::ReverseAndRescan);

  // With measured clearance, choose the side with more room.
  assert(gyroguardChooseScanAction(true, 700, true, 600, 450) ==
         GyroGuardScanAction::TurnLeft);
  assert(gyroguardChooseScanAction(true, 500, true, 650, 450) ==
         GyroGuardScanAction::TurnRight);

  return 0;
}
