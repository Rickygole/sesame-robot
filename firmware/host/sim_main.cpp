// sim_main.cpp -- headless simulation: GaitScheduler + IK + slew for 10
// simulated seconds at 50 Hz, no hardware. Dumps a CSV to stdout (one
// row per leg per tick) so gait quality can be inspected without a
// robot. Run via `make sim` (writes build/gait.csv).
//
// The residualMm column is the one to watch. Under the (theta, phi)
// planner it should be ~0 for every row: the planner works directly in
// the two coordinates the mechanism has, so a foot target is never
// off the reachable surface. The previous Cartesian planner produced
// -27.0 to -4.49 mm here, on a 55 mm leg, for all 2000 rows.
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
    case Leg::FR: return "FR";
    case Leg::RR: return "RR";
    case Leg::FL: return "FL";
    case Leg::RL: return "RL";
    default: return "??";
  }
}

// Placeholder body layout. Link lengths come from geometry.h's
// kPlaceholder* constants and are NOT measured -- see the TODO there.
void buildLegs(LegGeometry out[kLegCount]) {
  const float halfLen = 40.f;
  const float halfWid = 30.f;
  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    const bool front = (leg == Leg::FR || leg == Leg::FL);
    const bool right = (leg == Leg::FR || leg == Leg::RR);
    out[i].coxaMm = kPlaceholderCoxaMm;
    out[i].legMm = kPlaceholderLegMm;
    out[i].kneeBendDeg = kPlaceholderKneeBendDeg;
    out[i].hipInBody =
        Vec3(front ? halfLen : -halfLen, right ? halfWid : -halfWid, 0.f);
    out[i].lateralSign = right ? 1.f : -1.f;
    out[i].neutralYawDeg = kPlaceholderNeutralYawDeg;
  }
}

}  // namespace

int main() {
  LegGeometry geo[kLegCount];
  buildLegs(geo);

  GaitScheduler sched;
  sched.setGeometry(geo);
  sched.setGait(kCrawl);

  GaitCommand cmd;
  cmd.vxMmPerSec = 40.f;
  cmd.omegaDegPerSec = 0.f;
  cmd.bodyHeightMm = 0.5f * kPlaceholderLegMm;
  cmd.stepHeightMm = 0.15f * kPlaceholderLegMm;
  cmd.maxStrideMm = 0.f;
  sched.setCommand(cmd);

  MotionLimits limits;
  JointPose cur;
  for (uint8_t i = 0; i < kJointCount; ++i) cur.deg[i] = 0.f;

  printf(
      "t,phase,leg,hipDeg,kneeDeg,footHipX,footHipY,footHipZ,residualMm,"
      "reachable,inStance,throttle,bodyHeightMm,stepHeightMm,strideMm,"
      "stanceRadiusMm,maxScrubMmPerSec,maxLatDevMm,clamped\n");

  const float dt = 0.02f;  // the real 50 Hz tick
  const int kTicks = 500;  // 10 simulated seconds
  LegAngles ang[kLegCount];
  GaitReport rep;
  float throttle = 1.f;

  for (int t = 0; t < kTicks; ++t) {
    sched.step(dt, throttle, ang, rep);

    JointPose want = cur;
    for (uint8_t i = 0; i < kLegCount; ++i) {
      want.deg[logicalIndex(Leg(i), Joint::Hip)] = ang[i].hipDeg;
      want.deg[logicalIndex(Leg(i), Joint::Knee)] = ang[i].kneeDeg;
    }
    const SlewResult sr = applySlew(cur, want, limits, dt);
    cur = sr.applied;
    // Feed throttle back into the gait clock, so the gait stays coherent
    // under power limiting instead of the pose distorting.
    throttle = sr.throttle;

    for (uint8_t i = 0; i < kLegCount; ++i) {
      const Vec3 p = forwardKinematics(geo[i], ang[i]);
      const IkResult ik = solveFootPosition(geo[i], p);
      printf(
          "%.4f,%.4f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%d,%.4f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%.3f,%d\n",
          double(float(t) * dt), double(sched.phase()), legName(Leg(i)),
          double(ang[i].hipDeg), double(ang[i].kneeDeg), double(p.x),
          double(p.y), double(p.z), double(ik.residualMm),
          ik.reachable ? 1 : 0, sched.legInStance(Leg(i)) ? 1 : 0,
          double(sr.throttle), double(rep.bodyHeightMm),
          double(rep.stepHeightMm), double(rep.strideMm),
          double(rep.stanceRadiusMm), double(rep.maxScrubMmPerSec),
          double(rep.maxLateralDeviationMm), rep.clamped ? 1 : 0);
    }
  }
  return 0;
}
