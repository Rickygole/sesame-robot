// robot_config.h -- the physical dimensions of THIS robot.
//
// ============================ UNMEASURED ============================
// Every number here is a PLACEHOLDER. Nobody has measured the physical
// robot yet. The gait is exact with respect to whatever geometry it is
// given, so wrong numbers here produce a gait that is internally
// consistent but does not match the real machine -- the robot will walk,
// just not with the stride or stance you asked for.
//
// To correct them, measure with calipers and edit ONLY this file:
//   coxaMm  hip yaw axis  -> knee pitch axis
//   legMm   knee pitch axis -> foot contact point
//   halfLen body centre    -> hip axis, fore/aft
//   halfWid body centre    -> hip axis, left/right
//
// Nothing else in the firmware hardcodes a dimension, and the host test
// suite runs every gait invariant across a sweep of link lengths, so a
// correction here needs no other change.
// ====================================================================
#pragma once

#include "core/geometry.h"
#include "core/types.h"

namespace sesame {

// Body centre to hip axis. PLACEHOLDER -- measure.
constexpr float kHalfBodyLenMm = 40.f;
constexpr float kHalfBodyWidMm = 30.f;

// Fills `out` (indexed by logical Leg) with this robot's geometry.
inline void buildLegGeometry(core::LegGeometry out[core::kLegCount]) {
  for (uint8_t i = 0; i < core::kLegCount; ++i) {
    const core::Leg leg = core::Leg(i);
    const bool front = (leg == core::Leg::FR || leg == core::Leg::FL);
    const bool right = (leg == core::Leg::FR || leg == core::Leg::RR);
    out[i].coxaMm = core::kPlaceholderCoxaMm;
    out[i].legMm = core::kPlaceholderLegMm;
    out[i].kneeBendDeg = core::kPlaceholderKneeBendDeg;
    out[i].hipInBody = core::Vec3(front ? kHalfBodyLenMm : -kHalfBodyLenMm,
                                  right ? kHalfBodyWidMm : -kHalfBodyWidMm,
                                  0.f);
    out[i].lateralSign = right ? 1.f : -1.f;
    out[i].neutralYawDeg = core::kPlaceholderNeutralYawDeg;
  }
}

// Default drive parameters, as fractions of leg length so they stay
// sane if the link lengths are corrected.
// Body height as a fraction of leg length. 0.55 rather than 0.50 --
// standing slightly taller keeps the feet further from the body, which
// widens the support polygon and leaves more clearance for a swinging
// foot to pass a planted one.
constexpr float kDefaultBodyHeightFrac = 0.55f;

// Step height. Lowered from 0.15 after the first walk on hardware.
//
// This is the counter-intuitive one: a HIGHER lift is less stable, not
// more. Raising a foot on a 2-DOF leg also swings it inward (the foot
// travels on a torus -- height and reach are coupled), so a big lift
// pulls that corner's support inward exactly when the robot is already
// down to three feet. A small, brisk lift is steadier.
constexpr float kDefaultStepHeightFrac = 0.10f;

// If no Drive command arrives within this window, blend back to Stand.
// A robot that keeps walking because a controller disconnected mid-stride
// is a robot that walks off a table.
constexpr uint32_t kDriveWatchdogMs = 500;

}  // namespace sesame
