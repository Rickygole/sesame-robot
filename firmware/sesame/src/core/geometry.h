// geometry.h -- per-leg rigid-body geometry and forward kinematics.
//
// Frame conventions (see also ik.h):
//   Hip frame: origin at the hip YAW axis. x = forward, y = outward
//   lateral, z = up. Radial unit vector at yaw angle theta is
//   u(theta) = (sin theta, cos theta, 0); theta = 0 points straight out
//   laterally, theta > 0 swings the leg forward.
//
//   Body frame: origin at robot body center, x = forward, y = left,
//   z = up, shared across all four legs. Each LegGeometry carries
//   hipInBody (translation to that leg's hip yaw axis) and lateralSign,
//   chosen per-leg (+1 or -1) so that
//     hipFrame.y = lateralSign * (bodyFrame.y - hipInBody.y)
//   always gives the hip frame's outward-positive lateral coordinate.
//   This is how left/right mirroring of u(theta) is handled without
//   duplicating the IK/FK math per side.
//
// The leg itself is a RIGID printed link (no third servo). Its shape has
// a fixed internal bend, kneeBendDeg, baked in at manufacture. The knee
// SERVO angle (what actually gets commanded / calibrated) is offset from
// the geometric elevation angle phi (angle of the leg vector below
// horizontal; phi=0 -> leg straight out horizontal, phi=90deg -> foot
// straight down) by exactly that constant:
//
//   phi = kneeDeg_anatomical + kneeBendDeg
//
// i.e. kneeBendDeg is folded into the angle math once here, so every
// other layer (calibration, slew, gait) only ever sees/produces the
// anatomical "+ = foot down" knee degrees called for by JointPose.
#pragma once

#include "vec3.h"

namespace sesame {
namespace core {

// TODO(hardware): PLACEHOLDER link lengths. There is no physical robot
// to measure yet. These MUST be verified against the real, assembled
// CAD/hardware before any of this is trusted for actual motion. Do not
// ship these as final calibration data.
constexpr float kPlaceholderCoxaMm = 25.0f;
constexpr float kPlaceholderLegMm = 55.0f;
constexpr float kPlaceholderKneeBendDeg = 0.0f;

struct LegGeometry {
  float coxaMm;       // L_coxa: hip yaw axis to knee yaw axis, along u(theta)
  float legMm;        // L_leg: knee yaw axis to foot, rigid link length
  float kneeBendDeg;  // fixed internal link bend, see file comment above
  Vec3 hipInBody;      // hip yaw axis origin, expressed in body frame
  float lateralSign;   // +1 or -1; see frame-convention comment above
};

struct LegAngles {
  float hipDeg;   // anatomical: + = swing forward
  float kneeDeg;  // anatomical: + = foot down
};

// Radial unit vector in the hip frame at yaw angle theta (radians).
// u(0) points straight out laterally; u increases toward +x (forward)
// as theta increases, per the frame convention above.
Vec3 radialUnit(float thetaRad);

// Forward kinematics: anatomical leg angles -> foot position in the
// HIP frame. Always well-defined (no reachability question -- FK is
// total). See geometry.cpp for the closed-form.
Vec3 forwardKinematics(const LegGeometry& geo, LegAngles angles);

// Frame conversions between a leg's local hip frame and the shared body
// frame, using hipInBody / lateralSign.
Vec3 footBodyToHipFrame(const LegGeometry& geo, Vec3 footInBody);
Vec3 footHipFrameToBody(const LegGeometry& geo, Vec3 footInHipFrame);

}  // namespace core
}  // namespace sesame
