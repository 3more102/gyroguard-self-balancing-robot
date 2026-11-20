#include "imu_mpu6050.h"
#include "imu_calibration.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr float GYRO_LSB_PER_DPS = 131.0f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
}

bool MPU6050::begin() {
  initialized_ = false;
  filterInitialized_ = false;
  Wire.begin();
  Wire.setClock(400000UL);
  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(50);
  if (!writeRegister(REG_CONFIG, 0x03)) return false;       // DLPF ~44/42 Hz
  if (!writeRegister(REG_SMPLRT_DIV, 0x04)) return false;   // 200 Hz
  if (!writeRegister(REG_GYRO_CONFIG, 0x00)) return false;  // +/-250 deg/s
  if (!writeRegister(REG_ACCEL_CONFIG, 0x00)) return false; // +/-2 g
  delay(20);
  initialized_ = true;
  return true;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool MPU6050::readRegisters(uint8_t firstReg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(firstReg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t received = Wire.requestFrom(MPU6050_ADDRESS, length, (uint8_t)true);
  if (received != length) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) {
    if (!Wire.available()) return false;
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool MPU6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx,
                      int16_t &gy, int16_t &gz) {
  uint8_t b[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  ax = static_cast<int16_t>((static_cast<uint16_t>(b[0]) << 8) | b[1]);
  ay = static_cast<int16_t>((static_cast<uint16_t>(b[2]) << 8) | b[3]);
  az = static_cast<int16_t>((static_cast<uint16_t>(b[4]) << 8) | b[5]);
  gx = static_cast<int16_t>((static_cast<uint16_t>(b[8]) << 8) | b[9]);
  gy = static_cast<int16_t>((static_cast<uint16_t>(b[10]) << 8) | b[11]);
  gz = static_cast<int16_t>((static_cast<uint16_t>(b[12]) << 8) | b[13]);
  return true;
}

bool MPU6050::calibrate(uint16_t samples, float &zeroAngleDeg,
                        float &gyroBiasDps) {
  if (!initialized_ || samples < 100) return false;

  ImuCalibrationAccumulator calibration;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(ax, ay, az, gx, gy, gz)) return false;
    calibration.add(ax, ay, az, gx, gy, gz);
    delay(5);
  }

  if (!calibration.finish(samples, zeroAngleDeg, gyroBiasDps)) return false;

  filteredPitchDeg_ = 0.0f;
  filterInitialized_ = true;
  return true;
}

bool MPU6050::readPitch(float dtSeconds, float zeroAngleDeg, float gyroBiasDps,
                        float &pitchDeg, float &pitchRateDps) {
  if (!initialized_ || !(dtSeconds > 0.0f)) return false;
  int16_t ax, ay, az, gx, gy, gz;
  if (!readRaw(ax, ay, az, gx, gy, gz)) return false;

  const float accelAngle =
      atan2f(-static_cast<float>(ax), static_cast<float>(az)) * RAD_TO_DEG_F
      - zeroAngleDeg;
  pitchRateDps = static_cast<float>(gy) / GYRO_LSB_PER_DPS - gyroBiasDps;

  if (!filterInitialized_) {
    filteredPitchDeg_ = accelAngle;
    filterInitialized_ = true;
  } else {
    const float predicted = filteredPitchDeg_ + pitchRateDps * dtSeconds;
    filteredPitchDeg_ = COMPLEMENTARY_ALPHA * predicted
                      + (1.0f - COMPLEMENTARY_ALPHA) * accelAngle;
  }
  pitchDeg = filteredPitchDeg_;
  return true;
}
