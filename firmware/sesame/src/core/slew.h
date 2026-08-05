// slew.h -- rate limiting between the last commanded pose and a new
// desired pose. This is the safety-critical heart of the motion core:
// it is the ONLY thing standing between a bad command and a servo
// slamming into an end stop or the robot toppling itself over.
#pragma once

#include "pose.h"

namespace sesame {
namespace core {

struct MotionLimits {
  float maxDegPerSec = 200.f;       // per-joint max angular rate
  float maxTotalDegPerSec = 480.f;  // sum over all joints; proxy for total current draw
  float budgetScale = 1.0f;         // 0..1, cut by e.g. low battery
};

struct SlewResult {
  JointPose applied;
  float throttle;  // fraction of the per-joint-limited motion actually
                    // applied this step after the global budget, in
                    // [0,1]. 1.0 when the global budget wasn't binding.
                    // Feed this back into GaitScheduler::step() so the
                    // gait clock slows down coherently under power
                    // limiting instead of the pose distorting.
  bool limited;     // true if either the per-joint rate or the global
                     // budget clamped this step
};

// Semantics (all invariants, enforced by firmware/host/test_slew.cpp):
//  - Per-joint: |applied - current| <= maxDegPerSec * dt, every joint.
//  - Global: sum(|applied - current|) <= maxTotalDegPerSec * budgetScale * dt.
//  - When the global budget binds, ALL per-joint deltas (after the
//    per-joint rate clamp) are scaled by the SAME factor. No joint is
//    ever dropped or deferred individually -- that would distort the
//    pose shape and can topple the robot. Uniform scaling is a pure
//    time dilation: it preserves the direction of the pose-delta
//    vector exactly.
//  - Idempotent at target: desired == current => applied == current,
//    throttle == 1.
//  - Monotone convergence: repeated calls with a fixed desired pose
//    converge to it and never overshoot.
SlewResult applySlew(const JointPose& current, const JointPose& desired,
                      const MotionLimits& limits, float dt);

}  // namespace core
}  // namespace sesame
