#include "trajectory.h"

#include <cmath>

namespace sesame {
namespace core {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
}

float stanceDisplacement(float strideLen, float u) {
  return strideLen * (0.5f - u);
}

SwingSample swingTrajectory(float strideLen, float stepHeightMm, float u) {
  SwingSample out;
  out.s = strideLen * (u - std::sin(kTwoPi * u) / kTwoPi) - strideLen * 0.5f;
  out.h = stepHeightMm * 0.5f * (1.f - std::cos(kTwoPi * u));
  return out;
}

}  // namespace core
}  // namespace sesame
