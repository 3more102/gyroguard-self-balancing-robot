#include "../firmware/GyroGuard/navigation_logic.h"
#include <cassert>

int main() {
  const ScanDecision noEcho = decideScan(false, 0, false, 0, 450);
  assert(noEcho.action == ScanAction::Hold);
  assert(noEcho.turnDirection == 0);

  const ScanDecision leftClear = decideScan(true, 800, true, 200, 450);
  assert(leftClear.action == ScanAction::Turn);
  assert(leftClear.turnDirection == -1);

  const ScanDecision rightClear = decideScan(true, 150, true, 900, 450);
  assert(rightClear.action == ScanAction::Turn);
  assert(rightClear.turnDirection == 1);

  const ScanDecision onlyLeftKnown = decideScan(true, 700, false, 0, 450);
  assert(onlyLeftKnown.action == ScanAction::Turn);
  assert(onlyLeftKnown.turnDirection == -1);

  const ScanDecision onlyRightKnown = decideScan(false, 0, true, 700, 450);
  assert(onlyRightKnown.action == ScanAction::Turn);
  assert(onlyRightKnown.turnDirection == 1);

  const ScanDecision bothTight = decideScan(true, 300, true, 150, 450);
  assert(bothTight.action == ScanAction::Reverse);
  assert(bothTight.turnDirection == -1);

  const ScanDecision oneKnownTight = decideScan(true, 200, false, 0, 450);
  assert(oneKnownTight.action == ScanAction::Reverse);
  assert(oneKnownTight.turnDirection == -1);

  return 0;
}
