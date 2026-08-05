#include "slew.h"

#include <cmath>

namespace sesame {
namespace core {

namespace {
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

SlewResult applySlew(const JointPose& current, const JointPose& desired,
                      const MotionLimits& limits, float dt) {
  const float maxPerJoint = limits.maxDegPerSec * dt;
  const float globalBudget = limits.maxTotalDegPerSec * limits.budgetScale * dt;

  float perJointDelta[kJointCount];
  float sumAbsPerJoint = 0.f;
  bool perJointClamped = false;

  for (uint8_t i = 0; i < kJointCount; ++i) {
    const float raw = desired.deg[i] - current.deg[i];
    const float clamped = clampf(raw, -maxPerJoint, maxPerJoint);
    if (clamped != raw) {
      perJointClamped = true;
    }
    perJointDelta[i] = clamped;
    sumAbsPerJoint += std::fabs(clamped);
  }

  float factor = 1.f;
  if (sumAbsPerJoint > globalBudget && sumAbsPerJoint > 0.f) {
    factor = globalBudget / sumAbsPerJoint;
    if (factor < 0.f) factor = 0.f;
  }

  SlewResult result;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    result.applied.deg[i] = current.deg[i] + perJointDelta[i] * factor;
  }
  result.throttle = factor;
  result.limited = perJointClamped || (factor < 1.f);
  return result;
}

}  // namespace core
}  // namespace sesame
