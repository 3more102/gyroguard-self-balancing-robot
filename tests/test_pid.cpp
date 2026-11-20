#include "../firmware/GyroGuard/pid.h"
#include <cassert>
#include <cmath>
#include <limits>

static bool near(float a, float b, float eps = 1.0e-4f) {
  return std::fabs(a - b) <= eps;
}

int main() {
  PIDController proportional(2.0f, 0.0f, 0.0f, -10.0f, 10.0f);
  assert(near(proportional.update(3.0f, 0.02f), 6.0f));
  assert(near(proportional.update(20.0f, 0.02f), 10.0f));

  PIDController integral(0.0f, 1.0f, 0.0f, -5.0f, 5.0f);
  assert(near(integral.update(2.0f, 0.05f), 0.1f));
  integral.reset();
  assert(near(integral.update(2.0f, 0.05f), 0.1f));

  PIDController antiWindup(10.0f, 1.0f, 0.0f, -5.0f, 5.0f);
  assert(near(antiWindup.update(10.0f, 0.05f), 5.0f));
  assert(near(antiWindup.update(0.0f, 0.05f), 0.0f));

  PIDController derivative(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
  assert(near(derivative.update(0.0f, 0.01f, 2.5f, true), 2.5f));
  assert(near(derivative.update(1.0f, 0.0f), 0.0f));
  assert(near(derivative.update(1.0f, 0.5f), 0.0f));

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();

  PIDController invalidInput(0.0f, 1.0f, 0.0f, -5.0f, 5.0f);
  assert(near(invalidInput.update(2.0f, 0.05f), 0.1f));
  assert(near(invalidInput.update(nan, 0.05f), 0.0f));
  assert(near(invalidInput.update(0.0f, 0.05f), 0.0f));
  assert(near(invalidInput.update(infinity, 0.05f), 0.0f));

  PIDController invalidDerivative(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
  assert(near(invalidDerivative.update(0.0f, 0.01f, nan, true), 0.0f));
  assert(near(invalidDerivative.update(0.0f, 0.01f, 1.5f, true), 1.5f));

  return 0;
}
