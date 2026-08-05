// trajectory.h -- per-leg stance/swing foot trajectory shapes.
//
// Both functions work in a leg-local, 1D "direction of travel" frame:
// s is signed distance along the leg's velocity direction (s > 0 is
// ahead of the nominal stance point, s < 0 is behind it), h is height
// above the nominal stance plane (h >= 0, "up"). GaitScheduler maps
// (s,h) back into the body frame using the leg's travel direction and
// nominal anchor point.
//
// Both are parameterized by u in [0,1], the fraction of that phase
// (stance or swing) elapsed, and by strideLen, the total peak-to-peak
// foot excursion along the travel direction for one full gait cycle.
#pragma once

namespace sesame {
namespace core {

// Stance: straight line opposing the body's velocity. The foot is
// planted on the ground, so in the body frame it travels backward
// (from ahead of the anchor to behind it) at a constant rate as the
// body moves forward over it.
//   s(0) = +strideLen/2 (touchdown, ahead)
//   s(1) = -strideLen/2 (liftoff, behind)
float stanceDisplacement(float strideLen, float u);

struct SwingSample {
  float s;
  float h;
};

// Swing: cycloid arc carrying the foot from liftoff back to the next
// touchdown through the air.
//   s(0) = -strideLen/2, s(1) = +strideLen/2  (C0-continuous with stance)
//   ds/du(0) = ds/du(1) = 0  -- ~zero horizontal velocity at touchdown
//                               AND liftoff, which is what prevents
//                               scuffing.
//   h(0) = h(1) = 0, peaking at stepHeightMm around u=0.5 (half-cosine
//   bump), so height is also C0-continuous with the (h=0) stance phase.
SwingSample swingTrajectory(float strideLen, float stepHeightMm, float u);

}  // namespace core
}  // namespace sesame
