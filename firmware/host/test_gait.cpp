// test_gait.cpp -- invariants for the (theta, phi) gait planner.
//
// The PREVIOUS version of this suite passed a gait that never once
// produced a reachable foot position. Every test it had stayed INSIDE
// the planner: "phase wraps", ">=3 legs in stance", "trajectory is C0".
// All of those are true of a completely infeasible trajectory.
//
// The rule that follows, and the reason test 1 exists:
//
//   For any module that produces commands for a constrained system, at
//   least one test must assert those commands satisfy the system's
//   constraints.
//
// Test 1 crosses the planner/mechanism boundary: it takes the angles the
// planner actually emits, runs them forward through the kinematics, and
// asks the IK solver whether that point was reachable. Under the current
// design it is tautologically true -- which is exactly what makes it a
// good regression guard. It fires the instant anyone reintroduces
// Cartesian planning.
//
// Everything runs over a GEOMETRY SWEEP. Nobody has measured the real
// robot, so a fix that only works at the placeholder link lengths would
// be worthless.
#include <math.h>
#include <stdio.h>

#include "check.h"
#include "core/gait.h"
#include "core/geometry.h"
#include "core/ik.h"
#include "core/pose.h"
#include "core/slew.h"
#include "core/types.h"

using namespace sesame::core;

int g_fails = 0;

namespace {

struct GeomParams {
  float coxaMm;
  float legMm;
  float neutralYawDeg;
  float bodyHeightFrac;  // fraction of legMm
  float duty;
};

// Builds a plausible symmetric quadruped at the given link lengths.
// Deliberately parameterized: NO test may hardcode 25/55.
void buildLegs(const GeomParams& gp, LegGeometry out[kLegCount]) {
  const float halfLen = 40.f;
  const float halfWid = 30.f;
  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    const bool front = (leg == Leg::FR || leg == Leg::FL);
    const bool right = (leg == Leg::FR || leg == Leg::RR);
    out[i].coxaMm = gp.coxaMm;
    out[i].legMm = gp.legMm;
    out[i].kneeBendDeg = 0.f;
    out[i].hipInBody =
        Vec3(front ? halfLen : -halfLen, right ? halfWid : -halfWid, 0.f);
    out[i].lateralSign = right ? 1.f : -1.f;
    out[i].neutralYawDeg = gp.neutralYawDeg;
  }
}

const GeomParams kSweep[] = {
    {25.f, 55.f, 0.f, 0.5f, 0.75f},  // the placeholder, for continuity
    {10.f, 35.f, 0.f, 0.3f, 0.75f},
    {40.f, 85.f, 0.f, 0.8f, 0.75f},
    {25.f, 55.f, 30.f, 0.5f, 0.75f},
    {25.f, 55.f, 45.f, 0.5f, 0.75f},
    {10.f, 85.f, 30.f, 0.3f, 0.50f},  // trot duty
    {40.f, 35.f, 45.f, 0.8f, 0.50f},
};
const unsigned kSweepCount = sizeof(kSweep) / sizeof(kSweep[0]);

GaitDef gaitWithDuty(const GaitDef& base, float duty) {
  GaitDef d = base;
  d.dutyFactor = duty;
  return d;
}

// Forward velocity that lands the half-sweep at `targetHalfSweepDeg`,
// derived from the geometry rather than hardcoded.
//
// This matters: a 40 mm/s command needs a 30.5deg half-sweep on a 35mm
// leg (stance radius ~40mm) but only 16.9deg on a 55mm leg. A fixed
// velocity would silently exceed the mechanical sweep cap on small
// robots, so a "nothing was clamped" assertion would fail for a
// perfectly correct reason. Scaling with geometry is also what makes the
// sweep in main() a real geometry-independence test rather than seven
// runs of the same numbers.
float vxForHalfSweep(const GeomParams& gp, float bodyHeightMm, float freqHz,
                     float targetHalfSweepDeg) {
  const float kPiF = 3.14159265358979f;
  const float phiSt = asinf(bodyHeightMm / gp.legMm);
  const float rSt = gp.coxaMm + gp.legMm * cosf(phiSt);
  const float A = targetHalfSweepDeg * kPiF / 180.f;
  return A * 2.f * freqHz * rSt / gp.duty;
}

// A command that is comfortably INSIDE the physical limits for this
// geometry -- nothing should clamp.
GaitCommand baseCommand(const GeomParams& gp, float freqHz = kCrawl.freqHz) {
  GaitCommand cmd;
  cmd.bodyHeightMm = gp.bodyHeightFrac * gp.legMm;
  cmd.stepHeightMm = 0.15f * gp.legMm;
  cmd.omegaDegPerSec = 0.f;
  cmd.maxStrideMm = 0.f;
  cmd.vxMmPerSec = vxForHalfSweep(gp, cmd.bodyHeightMm, freqHz, 15.f);
  return cmd;
}

// --- 1. Reachability round-trip. THE test. --------------------------
void testReachabilityRoundTrip(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  GaitCommand cmd = baseCommand(gp);
  cmd.omegaDegPerSec = 15.f;
  s.setCommand(cmd);

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 512;
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      const Vec3 p = forwardKinematics(geo[i], ang[i]);
      const IkResult r = solveFootPosition(geo[i], p);
      // Exact by construction -- demand exactness, not "within a mm".
      CHECK_NEAR(r.residualMm, 0.f, 1e-4);
      CHECK(r.reachable);
    }
  }
}

// --- 2. The knobs do not lie ----------------------------------------
void testKnobsDoNotLie(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  GaitCommand cmd = baseCommand(gp);
  cmd.bodyHeightMm = 0.5f * gp.legMm;  // safely inside [0.15,0.95]
  cmd.stepHeightMm = 0.1f * gp.legMm;
  // Re-derive vx for the overridden height, so the sweep cap cannot bind.
  cmd.vxMmPerSec = vxForHalfSweep(gp, cmd.bodyHeightMm, kCrawl.freqHz, 15.f);
  s.setCommand(cmd);

  LegAngles ang[kLegCount];
  GaitReport rep;
  s.step(0.02f, 1.f, ang, rep);

  CHECK_NEAR(rep.bodyHeightMm, cmd.bodyHeightMm, 1e-4);
  CHECK_NEAR(rep.stepHeightMm, cmd.stepHeightMm, 1e-4);
  CHECK(!rep.clamped);
  CHECK(rep.stanceRadiusMm > 0.f);
}

// --- 3. No body bob: stance z is constant ---------------------------
void testNoBodyBob(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  s.setCommand(baseCommand(gp));

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 512;
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  float stanceZ[kLegCount];
  bool seen[kLegCount] = {false, false, false, false};

  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      if (!s.legInStance(Leg(i))) continue;
      const float z = forwardKinematics(geo[i], ang[i]).z;
      if (!seen[i]) {
        stanceZ[i] = z;
        seen[i] = true;
      } else {
        CHECK_NEAR(z, stanceZ[i], 1e-4);
      }
    }
  }
  for (uint8_t i = 0; i < kLegCount; ++i) {
    if (seen[i]) CHECK_NEAR(-stanceZ[i], rep.bodyHeightMm, 1e-3);
  }
}

// --- 4. Swing apex is exactly stepHeightMm above stance -------------
void testApexExact(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  GaitCommand cmd = baseCommand(gp);
  s.setCommand(cmd);

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 2048;
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  float maxZ = -1e30f;
  float stanceZ = 0.f;
  bool haveStance = false;

  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    const float z = forwardKinematics(geo[0], ang[0]).z;
    if (s.legInStance(Leg::FR)) {
      stanceZ = z;
      haveStance = true;
    } else if (z > maxZ) {
      maxZ = z;
    }
  }
  CHECK(haveStance);
  CHECK_NEAR(maxZ - stanceZ, cmd.stepHeightMm, 1e-3);
}

// --- 5. C1 closure around the WHOLE cycle, wrap included ------------
void testC1Closure(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  s.setCommand(baseCommand(gp));

  const int kSteps = 1024;
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  LegAngles ang[kLegCount];
  GaitReport rep;

  // Two full cycles, so the phase wrap lands in the interior of the
  // series rather than at its edge.
  const int kTotal = kSteps * 2;
  static float hip[2048 * 2];
  static float knee[2048 * 2];
  for (int step = 0; step < kTotal; ++step) {
    s.step(dt, 1.f, ang, rep);
    hip[step] = ang[0].hipDeg;
    knee[step] = ang[0].kneeDeg;
  }

  // A C1 curve sampled uniformly has second differences O(dt^2). A slope
  // discontinuity produces O(dt) -- orders of magnitude larger. Compare
  // the max against the mean so the bound scales with geometry instead
  // of being an absolute number tuned to one robot.
  float maxHip = 0.f, maxKnee = 0.f, sumHip = 0.f, sumKnee = 0.f;
  int n = 0;
  for (int i = 1; i + 1 < kTotal; ++i) {
    const float d2h = fabsf(hip[i + 1] - 2.f * hip[i] + hip[i - 1]);
    const float d2k = fabsf(knee[i + 1] - 2.f * knee[i] + knee[i - 1]);
    if (d2h > maxHip) maxHip = d2h;
    if (d2k > maxKnee) maxKnee = d2k;
    sumHip += d2h;
    sumKnee += d2k;
    ++n;
  }
  const float meanHip = sumHip / float(n);
  const float meanKnee = sumKnee / float(n);
  // Generous: we are catching a discontinuity (100x+), not ringing.
  if (meanHip > 1e-9f) CHECK(maxHip < meanHip * 60.f);
  if (meanKnee > 1e-9f) CHECK(maxKnee < meanKnee * 60.f);
}

// --- 6. Cross-module: the gait must fit the power budget ------------
// Neither module's own tests can see this. If the gait alone saturates
// the slew budget, the robot can never reach its commanded speed.
void testGaitFitsPowerBudget(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  GaitCommand cmd = baseCommand(gp);
  cmd.vxMmPerSec = 60.f;
  cmd.omegaDegPerSec = 20.f;
  s.setCommand(cmd);

  MotionLimits lim;
  LegAngles ang[kLegCount];
  GaitReport rep;
  JointPose cur;
  for (uint8_t i = 0; i < kJointCount; ++i) cur.deg[i] = 0.f;

  const float dt = 0.02f;  // the real 50Hz tick

  // Settle: the first ticks move from an arbitrary zero pose into the
  // gait, which is a transient, not steady state.
  for (int step = 0; step < 300; ++step) {
    s.step(dt, 1.f, ang, rep);
    JointPose want = cur;
    for (uint8_t i = 0; i < kLegCount; ++i) {
      want.deg[logicalIndex(Leg(i), Joint::Hip)] = ang[i].hipDeg;
      want.deg[logicalIndex(Leg(i), Joint::Knee)] = ang[i].kneeDeg;
    }
    cur = applySlew(cur, want, lim, dt).applied;
  }
  // Steady state: the gait must not saturate the budget.
  for (int step = 0; step < 400; ++step) {
    s.step(dt, 1.f, ang, rep);
    JointPose want = cur;
    for (uint8_t i = 0; i < kLegCount; ++i) {
      want.deg[logicalIndex(Leg(i), Joint::Hip)] = ang[i].hipDeg;
      want.deg[logicalIndex(Leg(i), Joint::Knee)] = ang[i].kneeDeg;
    }
    const SlewResult r = applySlew(cur, want, lim, dt);
    CHECK(r.throttle == 1.0f);
    cur = r.applied;
  }
}

// --- 7. Emitted angles stay within a sane joint range ---------------
void testJointLimits(const GeomParams& gp) {
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));
  GaitCommand cmd = baseCommand(gp);
  cmd.vxMmPerSec = 60.f;
  cmd.omegaDegPerSec = 30.f;
  s.setCommand(cmd);

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 512;
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      CHECK(ang[i].hipDeg >= -90.f && ang[i].hipDeg <= 90.f);
      CHECK(ang[i].kneeDeg >= -90.f && ang[i].kneeDeg <= 90.f);
    }
  }
}

// --- 8. Strafe stays dead -------------------------------------------
// Locks in the decision to CUT strafe rather than degrade it.
void testStrafeStaysDead(const GeomParams& gp) {
  GeomParams lateral = gp;
  lateral.neutralYawDeg = 0.f;  // legs straight out laterally
  LegGeometry geo[kLegCount];
  buildLegs(lateral, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(gaitWithDuty(kCrawl, gp.duty));

  GaitCommand cmd = baseCommand(lateral);
  cmd.vxMmPerSec = 50.f;
  cmd.omegaDegPerSec = 0.f;  // PURE forward
  s.setCommand(cmd);

  LegAngles ang[kLegCount];
  GaitReport rep;
  s.step(0.02f, 1.f, ang, rep);
  // Pure vx with laterally-neutral legs: the required foot velocity is
  // entirely tangential, so there is no radial (unachievable) component.
  CHECK_NEAR(rep.maxScrubMmPerSec, 0.f, 1e-3);

  // With omega, scrub is nonzero but must stay bounded and finite.
  cmd.omegaDegPerSec = 30.f;
  s.setCommand(cmd);
  s.step(0.02f, 1.f, ang, rep);
  CHECK(rep.maxScrubMmPerSec >= 0.f);
  CHECK(rep.maxScrubMmPerSec < 1000.f);
}

// --- 9. Static stability: crawl keeps >=3 feet down -----------------
void testCrawlStaticStability() {
  const GeomParams gp = kSweep[0];
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(kCrawl);  // duty 0.75
  s.setCommand(baseCommand(gp));

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 1024;  // fine sampling; a narrow violation window counts
  const float dt = 1.f / (kCrawl.freqHz * float(kSteps));
  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    int down = 0;
    for (uint8_t i = 0; i < kLegCount; ++i) {
      if (s.legInStance(Leg(i))) ++down;
    }
    CHECK(down >= 3);
  }
}

void testPhaseWrap() {
  const GeomParams gp = kSweep[0];
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(kCrawl);
  s.setCommand(baseCommand(gp));

  LegAngles ang[kLegCount];
  GaitReport rep;
  for (int step = 0; step < 5000; ++step) {
    s.step(0.02f, 1.f, ang, rep);
    CHECK(s.phase() >= 0.f && s.phase() < 1.f);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      const float lp = s.legPhase(Leg(i));
      CHECK(lp >= 0.f && lp < 1.f);
    }
  }
}

void testTrotAlsoReachable() {
  const GeomParams gp = kSweep[0];
  LegGeometry geo[kLegCount];
  buildLegs(gp, geo);
  GaitScheduler s;
  s.setGeometry(geo);
  s.setGait(kTrot);
  s.setCommand(baseCommand(gp));

  LegAngles ang[kLegCount];
  GaitReport rep;
  const int kSteps = 256;
  const float dt = 1.f / (kTrot.freqHz * float(kSteps));
  for (int step = 0; step < kSteps; ++step) {
    s.step(dt, 1.f, ang, rep);
    for (uint8_t i = 0; i < kLegCount; ++i) {
      const IkResult r =
          solveFootPosition(geo[i], forwardKinematics(geo[i], ang[i]));
      CHECK_NEAR(r.residualMm, 0.f, 1e-4);
    }
  }
}

}  // namespace

int main() {
  // --- 10. Every invariant, over the whole geometry sweep. ----------
  for (unsigned g = 0; g < kSweepCount; ++g) {
    const GeomParams& gp = kSweep[g];
    const int before = g_fails;

    testReachabilityRoundTrip(gp);
    testKnobsDoNotLie(gp);
    testNoBodyBob(gp);
    testApexExact(gp);
    testC1Closure(gp);
    testGaitFitsPowerBudget(gp);
    testJointLimits(gp);
    testStrafeStaysDead(gp);

    if (g_fails != before) {
      printf(
          "  ^ failures above at geometry coxa=%.0f leg=%.0f yaw=%.0f "
          "hFrac=%.2f duty=%.2f\n",
          double(gp.coxaMm), double(gp.legMm), double(gp.neutralYawDeg),
          double(gp.bodyHeightFrac), double(gp.duty));
    }
  }

  testCrawlStaticStability();
  testPhaseWrap();
  testTrotAlsoReachable();

  if (g_fails == 0) {
    printf("test_gait: all passed (%u geometries)\n", kSweepCount);
  }
  return g_fails ? 1 : 0;
}
