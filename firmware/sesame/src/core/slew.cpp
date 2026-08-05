#include "slew.h"

#include <cmath>

#include "mathutil.h"

namespace sesame {
namespace core {

SlewResult applySlew(const JointPose& current, const JointPose& desired,
                      const MotionLimits& limits, float dt) {
  // applySlew is the last stateful guard before a pose becomes servo
  // commands, so it must be total: EVERY input combination has to yield
  // a finite pose. The fail-safe posture is "hold the current pose" --
  // for a legged robot, refusing to move is always safer than moving to
  // an unknown angle.
  SlewResult hold;
  bool currentWasBad = false;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (isFiniteF(current.deg[i])) {
      hold.applied.deg[i] = current.deg[i];
    } else {
      hold.applied.deg[i] = 0.f;  // caller bug; neutral is the least-bad answer
      currentWasBad = true;
    }
  }
  hold.throttle = 0.f;
  hold.limited = true;

  if (!isFiniteF(dt) || dt <= 0.f || currentWasBad) {
    return hold;
  }
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (!isFiniteF(desired.deg[i])) {
      return hold;  // a NaN/Inf target must never propagate to a servo
    }
  }
  if (!isFiniteF(limits.maxDegPerSec) || !isFiniteF(limits.maxTotalDegPerSec) ||
      !isFiniteF(limits.budgetScale) || limits.maxDegPerSec < 0.f ||
      limits.maxTotalDegPerSec < 0.f || limits.budgetScale < 0.f) {
    return hold;
  }

  const float maxPerJoint = limits.maxDegPerSec * dt;
  const float globalBudget = limits.maxTotalDegPerSec * limits.budgetScale * dt;

  float perJointDelta[kJointCount];
  float sumAbsPerJoint = 0.f;
  bool perJointClamped = false;

  for (uint8_t i = 0; i < kJointCount; ++i) {
    const float raw = desired.deg[i] - current.deg[i];
    // raw is finite here (both operands checked above), so the NaN
    // fallback is unreachable; 0 = "do not move this joint" regardless.
    const float clamped = clampf(raw, -maxPerJoint, maxPerJoint, 0.f);
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
