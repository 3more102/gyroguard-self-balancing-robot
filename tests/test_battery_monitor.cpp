#include "../firmware/GyroGuard/battery_monitor.h"
#include "../firmware/GyroGuard/config.h"
#include <cassert>
#include <cstdint>

uint32_t fakeMillis = 0;
uint16_t fakeAnalogValue = 860;

int main() {
  BatteryMonitor monitor;
  monitor.begin();
  assert(!monitor.ready());
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // 860 ADC counts with a 5 V reference and 2:1 divider is about 8.4 V.
  assert(monitor.update(0));
  assert(monitor.ready());
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());
  assert(monitor.millivolts() == 8407U);

  // Sampling is rate-limited to 100 ms.
  fakeAnalogValue = 0;
  assert(!monitor.update(99));
  assert(!monitor.travelInhibited());

  // An out-of-range raw ADC sample must fail closed immediately even when
  // the filtered voltage still looks plausible.
  fakeAnalogValue = 1023;
  assert(monitor.update(100));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A valid sample at/above the recovery threshold clears the inhibit.
  fakeAnalogValue = 860;
  assert(monitor.update(200));
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());

  // A disconnected/shorted divider is invalid even while the IIR output
  // remains inside the nominal 5–9 V plausibility band.
  fakeAnalogValue = 0;
  assert(monitor.update(300));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // Recovery in the 6.8–7.0 V hysteresis band remains inhibited.
  fakeAnalogValue = 860;
  assert(monitor.update(400));
  assert(monitor.voltageValid());
  assert(monitor.travelInhibited());
  assert(monitor.lowBattery());
  assert(monitor.update(500));
  assert(!monitor.travelInhibited());

  // A sustained low-voltage condition eventually trips the travel inhibit.
  fakeAnalogValue = 665; // Approximately 6.5 V at the pack.
  assert(monitor.update(600));
  assert(!monitor.travelInhibited());
  assert(monitor.update(700));
  assert(!monitor.travelInhibited());
  assert(monitor.update(800));
  assert(!monitor.travelInhibited());
  assert(monitor.update(900));
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // Recovery requires the higher threshold; autonomy is still re-armed by
  // the application only after the operator explicitly sends 'a'.
  fakeAnalogValue = 860;
  assert(monitor.update(1000));
  assert(!monitor.travelInhibited());
  return 0;
}
