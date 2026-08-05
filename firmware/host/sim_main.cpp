// sim_main.cpp -- headless simulation: GaitScheduler + IK + slew for 10
// simulated seconds at 50 Hz, no hardware. Dumps a CSV to stdout (one
// row per leg per tick) so gait quality can be inspected without a
// robot. Run via `make sim` (writes build/gait.csv).
#include <cmath>
#include <cstdio>

#include "core/gait.h"
#include "core/geometry.h"
#include "core/ik.h"
#include "core/pose.h"
#include "core/slew.h"
#include "core/types.h"
#include "core/vec3.h"

using namespace sesame::core;

namespace {

const char* legName(Leg leg) {
  switch (leg) {
    case Leg::FR:
      return "FR";
    case Leg::RR:
      return "RR";
    case Leg::FL:
      return "FL";
    case Leg::RL:
      return "RL";
    default:
      return "??";
  }
}

// TODO(hardware): PLACEHOLDER hip mounting layout. No physical robot
// exists yet to measure it. The hips are modeled as mounted slightly
// inboard of the gait module's nominal (wider) foot stance width --
// plausible for a splayed-leg quadruped chassis -- purely so this demo
// simulation lands somewhere near the leg's reachable torus. This must
// be replaced with real CAD/hardware numbers before it means anything.
constexpr float kPlaceholderHipHalfLengthMm = 40.f;
constexpr float kPlaceholderHipHalfWidthMm = 10.f;

Vec3 hipMountInBody(Leg leg) {
  const float hx = kPlaceholderHipHalfLengthMm;
  const float hy = kPlaceholderHipHalfWidthMm;
  switch (leg) {
    case Leg::FR:
      return Vec3(hx, -hy, 0.f);
    case Leg::RR:
      return Vec3(-hx, -hy, 0.f);
    case Leg::FL:
      return Vec3(hx, hy, 0.f);
    case Leg::RL:
      return Vec3(-hx, hy, 0.f);
    default:
      return Vec3(0.f, 0.f, 0.f);
  }
}

// See geometry.h TODOs: link lengths are placeholders, no physical
// robot exists yet to measure them.
LegGeometry legGeometryFor(Leg leg) {
  LegGeometry g;
  g.coxaMm = kPlaceholderCoxaMm;
  g.legMm = kPlaceholderLegMm;
  g.kneeBendDeg = kPlaceholderKneeBendDeg;
  g.hipInBody = hipMountInBody(leg);
  // Right-side legs (FR,RR) sit at body -y; left-side (FL,RL) at +y.
  // lateralSign maps body Y into the hip frame's outward-positive Y.
  g.lateralSign = (leg == Leg::FR || leg == Leg::RR) ? -1.f : 1.f;
  return g;
}

}  // namespace

int main() {
  GaitScheduler sched;
  sched.setGait(kCrawl);

  GaitCommand cmd;
  cmd.vx = 60.f;          // mm/s forward
  cmd.vy = 0.f;
  cmd.omega = 0.f;
  cmd.bodyHeightMm = 48.f;
  cmd.stepHeightMm = 20.f;
  sched.setCommand(cmd);

  LegGeometry geo[kLegCount];
  for (uint8_t i = 0; i < kLegCount; ++i) {
    geo[i] = legGeometryFor(Leg(i));
  }

  MotionLimits limits;  // defaults

  JointPose current;  // zero pose
  float throttle = 1.f;

  const float dt = 1.f / 50.f;
  const int totalSteps = 10 * 50;  // 10 simulated seconds

  std::printf(
      "t,phase,leg,hipDeg,kneeDeg,footBodyX,footBodyY,footBodyZ,footHipX,"
      "footHipY,footHipZ,residualMm,reachable,throttle,totalDegPerSec\n");

  for (int step = 0; step < totalSteps; ++step) {
    const float t = float(step) * dt;

    Vec3 feetBody[kLegCount];
    sched.step(dt, throttle, feetBody);

    JointPose desired;
    IkResult ikResults[kLegCount];
    Vec3 feetHip[kLegCount];
    for (uint8_t i = 0; i < kLegCount; ++i) {
      const Leg leg = Leg(i);
      feetHip[i] = footBodyToHipFrame(geo[i], feetBody[i]);
      const IkResult ik = solveFootPosition(geo[i], feetHip[i]);
      ikResults[i] = ik;
      desired.deg[logicalIndex(leg, Joint::Hip)] = ik.angles.hipDeg;
      desired.deg[logicalIndex(leg, Joint::Knee)] = ik.angles.kneeDeg;
    }

    const SlewResult slewed = applySlew(current, desired, limits, dt);

    float totalDegPerSec = 0.f;
    for (uint8_t i = 0; i < kJointCount; ++i) {
      totalDegPerSec += std::fabs(slewed.applied.deg[i] - current.deg[i]);
    }
    totalDegPerSec /= dt;

    current = slewed.applied;
    throttle = slewed.throttle;

    for (uint8_t i = 0; i < kLegCount; ++i) {
      const Leg leg = Leg(i);
      const IkResult& ik = ikResults[i];
      std::printf(
          "%.4f,%.4f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%."
          "4f,%.3f\n",
          t, sched.phase(), legName(leg),
          current.deg[logicalIndex(leg, Joint::Hip)],
          current.deg[logicalIndex(leg, Joint::Knee)], feetBody[i].x,
          feetBody[i].y, feetBody[i].z, feetHip[i].x, feetHip[i].y,
          feetHip[i].z, ik.residualMm, ik.reachable ? 1 : 0, throttle,
          totalDegPerSec);
    }
  }

  return 0;
}
