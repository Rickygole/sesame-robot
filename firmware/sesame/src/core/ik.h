// ik.h -- closed-form 2-DOF-per-leg inverse kinematics.
//
// HONESTY NOTE: each leg has exactly two servos (hip yaw, knee elevation)
// driving a rigid link of fixed length. The set of reachable foot
// positions is therefore NOT all of 3D space -- it is a TORUS: a circle
// of radius legMm swept about the hip's vertical (yaw) axis at radius
// coxaMm. That is a 2D surface embedded in 3D. You get to choose exactly
// two of {fore-aft, lateral, height} for the foot; the third is
// dependent on the other two once you commit to being ON the torus.
//
// solveFootPosition() does NOT silently satisfy an unreachable request.
// It projects the request radially onto the torus (same yaw angle,
// closest point on the leg's great circle in the (rho, z) half-plane)
// and always reports residualMm, the signed leftover distance between
// the requested point and the reachable surface along that great
// circle. residualMm == 0 (within kIkReachEpsilonMm) iff the request was
// exactly reachable; reachable is a convenience bool for that check.
// Callers that need exact reachability (e.g. swing trajectories with a
// known yaw+depth) should use solveSwingDepth instead, which is exact
// by construction.
#pragma once

#include "geometry.h"
#include "vec3.h"

namespace sesame {
namespace core {

// Residuals within this tolerance are treated as "exactly reachable".
// Chosen well below any servo/print tolerance; purely a float-noise
// guard, not a physical slop allowance.
constexpr float kIkReachEpsilonMm = 0.01f;

struct IkResult {
  LegAngles angles;
  float residualMm;  // reach - legMm, signed; 0 => p is exactly on the torus
  bool reachable;     // |residualMm| < kIkReachEpsilonMm
};

// Closed-form IK. footInHipFrame is in the leg's local hip frame (see
// geometry.h). See file comment above for the reachability caveat.
//
// STATUS: this is a TEST ORACLE (used by test_ik.cpp's round-trip check
// and by a future manual foot-placement CLI command), not part of the
// runtime gait path. GaitScheduler::step() never asks "what angles put
// the foot at this (x,y,z)" -- it asks "what angles put the foot at this
// yaw, at this elevation angle" (anglesFromYawPitch, below), which is
// exactly reachable by construction and needs no projection or residual.
IkResult solveFootPosition(const LegGeometry& geo, Vec3 footInHipFrame);

// Converts an already-chosen (yaw, elevation) pair directly to anatomical
// servo angles. Pure unit conversion + the fixed link-bend offset -- NO
// trigonometry, because yawDeg and pitchRad (phi, radians below
// horizontal) are already exactly the two angles the leg's two servos
// command. There is no reachability question: every (yawDeg, pitchRad)
// pair names a point ON the torus, so residual is zero by construction.
// This is the one primitive the gait planner is built on.
LegAngles anglesFromYawPitch(const LegGeometry& geo, float yawDeg, float pitchRad);

// mm-facing wrapper over anglesFromYawPitch: picks a point ON the torus
// for a given yaw and "how far below horizontal" depth (mm, along -z at
// the resulting radial distance). Always exact: residual is 0 by
// construction, no reachability question.
LegAngles solveSwingDepth(const LegGeometry& geo, float yawDeg, float depthMm);

}  // namespace core
}  // namespace sesame
