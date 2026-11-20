#pragma once

#include <Arduino.h>

class MPU6050 {
public:
  bool begin();
  bool calibrate(uint16_t samples, float &zeroAngleDeg, float &gyroBiasDps);
  bool readPitch(float dtSeconds, float zeroAngleDeg, float gyroBiasDps,
                 float &pitchDeg, float &pitchRateDps);

private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t firstReg, uint8_t *buffer, uint8_t length);
  bool readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx,
               int16_t &gy, int16_t &gz);
  bool initialized_;
  bool filterInitialized_;
  float filteredPitchDeg_;
};
