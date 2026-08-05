#include <cstdio>

#include "check.h"
#include "core/pose.h"
#include "core/types.h"

int g_fails = 0;

using sesame::core::JointPose;
using sesame::core::kJointCount;

static void testLerpEndpoints() {
  JointPose a;
  JointPose b;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    a.deg[i] = float(i) * 3.5f - 10.f;
    b.deg[i] = float(i) * -2.0f + 40.f;
  }

  const JointPose at0 = JointPose::lerp(a, b, 0.f);
  const JointPose at1 = JointPose::lerp(a, b, 1.f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK_NEAR(at0.deg[i], a.deg[i], 1e-6f);
    CHECK_NEAR(at1.deg[i], b.deg[i], 1e-6f);
  }
}

static void testLerpMonotone() {
  JointPose a;
  JointPose b;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    a.deg[i] = -20.f;
    b.deg[i] = 60.f;
  }

  float prev[kJointCount];
  for (uint8_t i = 0; i < kJointCount; ++i) prev[i] = a.deg[i];

  const int kSteps = 20;
  for (int s = 1; s <= kSteps; ++s) {
    const float t = float(s) / float(kSteps);
    const JointPose p = JointPose::lerp(a, b, t);
    for (uint8_t i = 0; i < kJointCount; ++i) {
      // b > a for every joint here, so lerp must be non-decreasing in t.
      CHECK(p.deg[i] >= prev[i] - 1e-6f);
      prev[i] = p.deg[i];
    }
  }
}

static void testLerpMidpoint() {
  JointPose a;
  JointPose b;
  a.deg[0] = 0.f;
  b.deg[0] = 10.f;
  const JointPose mid = JointPose::lerp(a, b, 0.5f);
  CHECK_NEAR(mid.deg[0], 5.f, 1e-6f);
}

int main() {
  testLerpEndpoints();
  testLerpMonotone();
  testLerpMidpoint();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
