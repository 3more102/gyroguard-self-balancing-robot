#pragma once

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float outputMin, float outputMax);
  void reset();
  float update(float error, float dtSeconds, float derivativeMeasurement = 0.0f,
               bool useDerivativeMeasurement = false);

private:
  float kp_;
  float ki_;
  float kd_;
  float outputMin_;
  float outputMax_;
  float integral_;
  float previousError_;
  bool hasPrevious_;
};
