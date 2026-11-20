#pragma once

#include <Arduino.h>

// Arduino Uno R3 / classic Nano (ATmega328P, 16 MHz).
// IMU orientation: PCB flat; +X toward robot front, +Y toward robot left,
// +Z upward. Pitch is rotation around +Y.

// TB6612FNG motor driver
constexpr uint8_t PIN_LEFT_PWM = 5;
constexpr uint8_t PIN_LEFT_IN1 = 8;
constexpr uint8_t PIN_LEFT_IN2 = 10;
constexpr uint8_t PIN_RIGHT_PWM = 6;
constexpr uint8_t PIN_RIGHT_IN1 = 11;
constexpr uint8_t PIN_RIGHT_IN2 = 12;
constexpr uint8_t PIN_MOTOR_STBY = A3;

// Quadrature encoders: interrupt on channel A, sample channel B (x2 decoding).
constexpr uint8_t PIN_LEFT_ENC_A = 2;   // INT0
constexpr uint8_t PIN_LEFT_ENC_B = 4;
constexpr uint8_t PIN_RIGHT_ENC_A = 3;  // INT1
constexpr uint8_t PIN_RIGHT_ENC_B = 7;

// HC-SR04 Echo on A1 uses PCINT9. Change sonar.cpp if this pin is changed.
constexpr uint8_t PIN_SONAR_TRIG = A0;
constexpr uint8_t PIN_SONAR_ECHO = A1;
constexpr uint8_t PIN_SERVO = 9;        // Servo library uses Timer1

// Motor and encoder polarity. Verify with wheels raised.
constexpr int8_t LEFT_MOTOR_SIGN = +1;
constexpr int8_t RIGHT_MOTOR_SIGN = +1;
constexpr int8_t LEFT_ENCODER_SIGN = +1;
constexpr int8_t RIGHT_ENCODER_SIGN = +1;

// x2 counts per wheel revolution = channel-A cycles per motor revolution
// * gearbox ratio * 2. This default is an example; measure your motors.
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 660.0f;
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265f * WHEEL_DIAMETER_MM;

// Real-time scheduling
constexpr uint32_t CONTROL_PERIOD_US = 5000UL;       // 200 Hz
constexpr uint32_t VELOCITY_PERIOD_US = 20000UL;     // 50 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 100UL;      // 10 Hz
constexpr uint32_t MAX_CONTROL_GAP_US = 20000UL;

// MPU-6050
constexpr uint8_t MPU6050_ADDRESS = 0x68; // AD0 low; use 0x69 if AD0 high
constexpr uint16_t IMU_CALIBRATION_SAMPLES = 500;
constexpr float IMU_MAX_CALIBRATION_ANGLE_STDDEV_DEG = 1.5f;
constexpr float IMU_MAX_CALIBRATION_GYRO_STDDEV_DPS = 2.0f;
constexpr float IMU_MIN_CALIBRATION_GRAVITY_G = 0.75f;
constexpr float IMU_MAX_CALIBRATION_GRAVITY_G = 1.25f;
constexpr uint32_t IMU_I2C_TIMEOUT_US = 3000UL;
constexpr float COMPLEMENTARY_ALPHA = 0.98f;
constexpr float MAX_SAFE_TILT_DEG = 32.0f;

// Controller gains are initial tuning values, not guaranteed stable.
constexpr float BALANCE_KP = 18.0f;       // PWM / degree
constexpr float BALANCE_KI = 0.0f;        // PWM / (degree second)
constexpr float BALANCE_KD = 0.75f;       // PWM / (degree/second)
constexpr float BALANCE_OUTPUT_LIMIT = 210.0f;

constexpr float VELOCITY_KP = 0.035f;     // degree / (mm/s)
constexpr float VELOCITY_KI = 0.008f;     // degree / mm
constexpr float MAX_LEAN_DEG = 5.0f;
constexpr float MAX_TARGET_SPEED_MM_S = 180.0f;

// Navigation distances are millimetres.
constexpr bool AUTO_DRIVE_ON_BOOT = false;
constexpr float CRUISE_SPEED_MM_S = 100.0f;
constexpr float OBSTACLE_STOP_MM = 320.0f;
constexpr float OBSTACLE_CLEAR_MM = 450.0f;
constexpr uint16_t SONAR_MAX_RANGE_MM = 2500;
constexpr uint16_t SONAR_MIN_RANGE_MM = 30;
constexpr uint8_t SERVO_CENTER_DEG = 90;
constexpr uint8_t SERVO_LEFT_DEG = 145;
constexpr uint8_t SERVO_RIGHT_DEG = 35;
constexpr uint16_t SERVO_SETTLE_MS = 250;
constexpr uint16_t SONAR_RETRY_MS = 65;
constexpr uint16_t FRONT_RANGE_MAX_AGE_MS = 150;
constexpr uint16_t TURN_DURATION_MS = 650;
constexpr float TURN_BIAS_PWM = 38.0f;

// If scan directions are reversed by your bracket, swap LEFT and RIGHT angles.
constexpr unsigned long SERIAL_BAUD = 115200UL;
