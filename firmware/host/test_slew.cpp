#include <cmath>
#include <cstdio>

#include "check.h"
#include "core/pose.h"
#include "core/slew.h"
#include "core/types.h"

int g_fails = 0;

using sesame::core::JointPose;
using sesame::core::kJointCount;
using sesame::core::MotionLimits;
using sesame::core::SlewResult;
using sesame::core::applySlew;

static JointPose makePose(float v) {
  JointPose p;
  for (uint8_t i = 0; i < kJointCount; ++i) p.deg[i] = v;
  return p;
}

static void testIdempotentAtTarget() {
  const JointPose current = makePose(12.3f);
  MotionLimits limits;
  const SlewResult r = applySlew(current, current, limits, 0.02f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK_NEAR(r.applied.deg[i], current.deg[i], 1e-6f);
  }
  CHECK_NEAR(r.throttle, 1.f, 1e-6f);
  CHECK(!r.limited);
}

static void testPerJointRateNeverExceeded() {
  JointPose current = makePose(0.f);
  JointPose desired = makePose(0.f);
  // Wildly different, some huge, some small deltas per joint.
  const float deltas[kJointCount] = {1000.f, -500.f, 0.1f, 0.f,
                                      45.f,   -0.05f, 300.f, -1000.f};
  for (uint8_t i = 0; i < kJointCount; ++i) desired.deg[i] = deltas[i];

  MotionLimits limits;
  limits.maxDegPerSec = 90.f;
  limits.maxTotalDegPerSec = 1000000.f;  // effectively no global limit
  const float dt = 0.02f;
  const SlewResult r = applySlew(current, desired, limits, dt);

  const float maxPerJoint = limits.maxDegPerSec * dt;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    const float d = std::fabs(r.applied.deg[i] - current.deg[i]);
    CHECK(d <= maxPerJoint + 1e-4f);
  }
}

static void testGlobalBudgetNeverExceeded() {
  JointPose current = makePose(0.f);
  JointPose desired = makePose(50.f);  // every joint wants to move 50 deg

  MotionLimits limits;
  limits.maxDegPerSec = 1000.f;  // per-joint effectively unlimited
  limits.maxTotalDegPerSec = 100.f;
  limits.budgetScale = 1.f;
  const float dt = 0.1f;
  const SlewResult r = applySlew(current, desired, limits, dt);

  float sumAbs = 0.f;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    sumAbs += std::fabs(r.applied.deg[i] - current.deg[i]);
  }
  const float budget = limits.maxTotalDegPerSec * limits.budgetScale * dt;
  CHECK(sumAbs <= budget + 1e-3f);
  CHECK(r.limited);
  CHECK(r.throttle > 0.f && r.throttle <= 1.f);
}

static void testThrottleRangeAndUnlimitedCase() {
  JointPose current = makePose(0.f);
  JointPose desired = makePose(1.f);  // tiny motion, nothing should bind
  MotionLimits limits;
  const SlewResult r = applySlew(current, desired, limits, 0.02f);
  CHECK_NEAR(r.throttle, 1.f, 1e-6f);
  CHECK(!r.limited);
}

static void testUniformScalingPreservesDirection() {
  // Pick deltas within per-joint limits so only the global budget binds;
  // then the applied delta vector must be an exact positive scalar
  // multiple of the raw desired delta vector (anti-topple invariant).
  JointPose current = makePose(0.f);
  JointPose desired;
  const float raw[kJointCount] = {10.f, -20.f, 30.f, -5.f, 15.f, -10.f, 0.f, 8.f};
  for (uint8_t i = 0; i < kJointCount; ++i) desired.deg[i] = raw[i];

  MotionLimits limits;
  limits.maxDegPerSec = 10000.f;  // per-joint never binds
  limits.maxTotalDegPerSec = 50.f;
  limits.budgetScale = 1.f;
  const float dt = 1.f;
  const SlewResult r = applySlew(current, desired, limits, dt);
  CHECK(r.limited);

  // Find the scale factor from a nonzero component, then check every
  // other component uses the exact same factor (same direction).
  float factor = -1.f;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (raw[i] != 0.f) {
      factor = r.applied.deg[i] / raw[i];
      break;
    }
  }
  CHECK(factor > 0.f && factor < 1.f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    if (raw[i] != 0.f) {
      CHECK_NEAR(r.applied.deg[i] / raw[i], factor, 1e-4f);
    } else {
      CHECK_NEAR(r.applied.deg[i], 0.f, 1e-6f);
    }
  }
}

static void testMonotoneConvergence() {
  JointPose current = makePose(0.f);
  const JointPose desired = makePose(37.f);
  MotionLimits limits;
  limits.maxDegPerSec = 50.f;
  limits.maxTotalDegPerSec = 50.f * kJointCount;  // no global binding
  const float dt = 0.05f;

  float prevErr = std::fabs(desired.deg[0] - current.deg[0]);
  bool reached = false;
  for (int step = 0; step < 1000; ++step) {
    const SlewResult r = applySlew(current, desired, limits, dt);
    const float err = std::fabs(desired.deg[0] - r.applied.deg[0]);
    CHECK(err <= prevErr + 1e-5f);  // never grows
    prevErr = err;
    current = r.applied;
    if (err < 1e-5f) {
      reached = true;
      break;
    }
  }
  CHECK(reached);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK_NEAR(current.deg[i], desired.deg[i], 1e-3f);
  }
}

static void testNeverOvershoots() {
  JointPose current = makePose(0.f);
  const JointPose desired = makePose(5.f);  // small target relative to rate
  MotionLimits limits;
  limits.maxDegPerSec = 1000.f;
  limits.maxTotalDegPerSec = 1000.f * kJointCount;
  const SlewResult r = applySlew(current, desired, limits, 1.f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK(r.applied.deg[i] <= desired.deg[i] + 1e-4f);
  }
}

int main() {
  testIdempotentAtTarget();
  testPerJointRateNeverExceeded();
  testGlobalBudgetNeverExceeded();
  testThrottleRangeAndUnlimitedCase();
  testUniformScalingPreservesDirection();
  testMonotoneConvergence();
  testNeverOvershoots();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
