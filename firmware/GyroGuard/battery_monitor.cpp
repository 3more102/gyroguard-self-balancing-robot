#include "battery_monitor.h"
#include "battery_policy.h"
#include "config.h"

void BatteryMonitor::begin() {
  pinMode(PIN_BATTERY_SENSE, INPUT);
  lastSampleMs_ = 0;
  filteredMillivolts_ = 0;
  initialized_ = false;
  travelInhibited_ = true; // Fail closed until a valid recovery-level reading.
  sampleValid_ = false;
}

bool BatteryMonitor::update(uint32_t nowMs) {
  if (initialized_ &&
      static_cast<uint32_t>(nowMs - lastSampleMs_) < BATTERY_SAMPLE_PERIOD_MS) {
    return false;
  }

  // Discard one conversion after selecting A2, then use the settled reading.
  (void)analogRead(PIN_BATTERY_SENSE);
  const uint16_t raw = static_cast<uint16_t>(analogRead(PIN_BATTERY_SENSE));
  const uint16_t sampleMv = batteryMillivoltsFromAdc(
      raw, BATTERY_ADC_REFERENCE_MV, BATTERY_DIVIDER_RATIO);
  const bool rawSampleValid = batteryVoltagePlausible(
      sampleMv, BATTERY_MIN_VALID_MV, BATTERY_MAX_VALID_MV);

  if (!initialized_) {
    filteredMillivolts_ = sampleMv;
  } else {
    const uint32_t divisor = BATTERY_FILTER_DIVISOR;
    const uint32_t weighted =
        (divisor - 1U) * filteredMillivolts_ + sampleMv + divisor / 2U;
    filteredMillivolts_ = static_cast<uint16_t>(weighted / divisor);
  }

  const bool filteredValid = batteryVoltagePlausible(
      filteredMillivolts_, BATTERY_MIN_VALID_MV, BATTERY_MAX_VALID_MV);
  sampleValid_ = rawSampleValid && filteredValid;
  travelInhibited_ = batteryTravelInhibited(
      travelInhibited_, filteredMillivolts_, sampleValid_,
      LOW_BATTERY_INHIBIT_MV, LOW_BATTERY_RECOVERY_MV);
  lastSampleMs_ = nowMs;
  initialized_ = true;
  return true;
}

bool BatteryMonitor::ready() const {
  return initialized_;
}

bool BatteryMonitor::lowBattery() const {
  return initialized_ && voltageValid() && travelInhibited_;
}

bool BatteryMonitor::voltageValid() const {
  return initialized_ && sampleValid_ && batteryVoltagePlausible(
      filteredMillivolts_, BATTERY_MIN_VALID_MV, BATTERY_MAX_VALID_MV);
}

bool BatteryMonitor::travelInhibited() const {
  return !voltageValid() || travelInhibited_;
}

uint16_t BatteryMonitor::millivolts() const {
  return initialized_ ? filteredMillivolts_ : 0U;
}
