// test_envelope.cpp -- invariants for the safety-envelope layer that
// gates and clamps a command BEFORE the gait ever sees it.
//
// Same house rule as test_gait.cpp:
//
//   For any module that produces commands for a constrained system, at
//   least one test must assert those commands satisfy the system's
//   constraints.
//
// Envelope's whole job is producing GaitCommands for GaitScheduler (the
// constrained system), so several tests here cross that boundary: they
// take what Envelope::apply() actually emits, run it through the real
// gait, and check the gait either didn't need to clamp further (single
// axis) or, when it does need to (combined vx+omega, a case the header
// documents as NOT independently guaranteed), that the emitted angles
// are still exactly reachable -- i.e. gait.cpp's own backstop clamp
// really is a backstop, not a documentation fiction.
//
// Everything runs over a GEOMETRY SWEEP: coxa {10,25,40} x leg {35,55,85}
// x duty {0.5,0.75}. Nobody has measured the real robot, so a module
// that only works at the placeholder link lengths is worthless. NO test
// may hardcode a single fixed 25/55 pair as if it were the only shape
// that mattered.
#include <math.h>
#include <stdio.h>

#include "check.h"
#include "core/envelope.h"
#include "core/gait.h"
#include "core/geometry.h"
#include "core/ik.h"
#include "core/types.h"

using namespace sesame::core;

int g_fails = 0;

namespace {

constexpr float kPiF = 3.14159265358979323846f;
inline float degToRadF(float deg) { return deg * (kPiF / 180.f); }

// --- Geometry sweep ---------------------------------------------------

const float kCoxaSweep[] = {10.f, 25.f, 40.f};
const float kLegSweep[] = {35.f, 55.f, 85.f};
const float kDutySweep[] = {0.5f, 0.75f};

struct SweepPoint {
  float coxaMm;
  float legMm;
  float duty;
};

// Full cross product of the three axes named in the spec.
SweepPoint kSweep[3 * 3 * 2];
unsigned kSweepCount = 0;

void buildSweep() {
  kSweepCount = 0;
  for (unsigned c = 0; c < 3; ++c) {
    for (unsigned l = 0; l < 3; ++l) {
      for (unsigned d = 0; d < 2; ++d) {
        kSweep[kSweepCount].coxaMm = kCoxaSweep[c];
        kSweep[kSweepCount].legMm = kLegSweep[l];
        kSweep[kSweepCount].duty = kDutySweep[d];
        ++kSweepCount;
      }
    }
  }
}

GaitDef gaitWithDuty(float duty) {
  GaitDef d = kCrawl;
  d.dutyFactor = duty;
  return d;
}

// A realistic symmetric quadruped layout (mirrors test_gait.cpp's
// buildLegs), used whenever a test needs to run the actual scheduler.
void buildLegs(float coxaMm, float legMm, LegGeometry out[kLegCount]) {
  const float halfLen = 40.f;
  const float halfWid = 30.f;
  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    const bool front = (leg == Leg::FR || leg == Leg::FL);
    const bool right = (leg == Leg::FR || leg == Leg::RR);
    out[i].coxaMm = coxaMm;
    out[i].legMm = legMm;
    out[i].kneeBendDeg = 0.f;
    out[i].hipInBody =
        Vec3(front ? halfLen : -halfLen, right ? halfWid : -halfWid, 0.f);
    out[i].lateralSign = right ? 1.f : -1.f;
    out[i].neutralYawDeg = 0.f;
  }
}

// Degenerate layout with every hip AT the body origin. The header's
// derivation of omegaMaxDegPerSec is exact only in this case (no
// hipInBody moment-arm term) -- this is what lets test 1 assert the
// derived cap reproduces A == kMaxHalfSweepDeg exactly, for BOTH axes,
// rather than just vx.
void buildLegsAtOrigin(float coxaMm, float legMm, LegGeometry out[kLegCount]) {
  for (uint8_t i = 0; i < kLegCount; ++i) {
    out[i].coxaMm = coxaMm;
    out[i].legMm = legMm;
    out[i].kneeBendDeg = 0.f;
    out[i].hipInBody = Vec3(0.f, 0.f, 0.f);
    out[i].lateralSign = 1.f;
    out[i].neutralYawDeg = 0.f;
  }
}

float achievedHeight(float legMm) {
  // Comfortably inside [0.15, 0.95] * legMm for every geometry in the
  // sweep, so height itself never binds while exercising vx/omega.
  return 0.5f * legMm;
}

// --- 1. Derived caps reproduce the gait's own equation -----------------
void testDerivedCapsMatchGaitFormula(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegsAtOrigin(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);
  const float h = achievedHeight(sp.legMm);

  const EnvelopeLimits limits = computeLimits(geo[0], def, h);
  CHECK(limits.vxMaxMmPerSec > 0.f);
  CHECK(limits.omegaMaxDegPerSec > 0.f);
  CHECK(limits.stanceRadiusMm > 0.f);

  const float halfSweepRad = degToRadF(kMaxHalfSweepDeg);

  // vx axis: command exactly vxMax with omega == 0, feed through the
  // real scheduler, recover A from the achieved stride (strideMm ==
  // 2*A*rSt, see gait.cpp/gait.h) and check it lands on the mechanical
  // cap.
  {
    GaitScheduler s;
    s.setGeometry(geo);
    s.setGait(def);
    GaitCommand cmd;
    cmd.bodyHeightMm = h;
    cmd.stepHeightMm = 0.1f * sp.legMm;
    cmd.vxMmPerSec = limits.vxMaxMmPerSec;
    cmd.omegaDegPerSec = 0.f;
    s.setCommand(cmd);
    LegAngles ang[kLegCount];
    GaitReport rep;
    s.step(0.001f, 1.f, ang, rep);
    // NOTE: not asserting !rep.clamped here -- commanding EXACTLY the
    // boundary means the forward/inverse round trip through the sweep
    // formula can land a ULP or two on either side of
    // kMaxHalfSweepDeg, and gait.cpp's own clamp flag is a strict
    // inequality, so it can trip on pure float noise at the boundary.
    // The actual claim ("this cap reproduces A == kMaxHalfSweepDeg") is
    // what CHECK_NEAR below verifies.
    const float A = rep.strideMm / (2.f * rep.stanceRadiusMm);
    CHECK_NEAR(A, halfSweepRad, 1e-4);
  }

  // omega axis: command exactly omegaMax with vx == 0. Exact only
  // because buildLegsAtOrigin puts every hip at the body origin -- see
  // the comment there and in envelope.h's derivation.
  {
    GaitScheduler s;
    s.setGeometry(geo);
    s.setGait(def);
    GaitCommand cmd;
    cmd.bodyHeightMm = h;
    cmd.stepHeightMm = 0.1f * sp.legMm;
    cmd.vxMmPerSec = 0.f;
    cmd.omegaDegPerSec = limits.omegaMaxDegPerSec;
    s.setCommand(cmd);
    LegAngles ang[kLegCount];
    GaitReport rep;
    s.step(0.001f, 1.f, ang, rep);
    // Same float-boundary note as the vx block above.
    const float A = rep.strideMm / (2.f * rep.stanceRadiusMm);
    CHECK_NEAR(A, halfSweepRad, 1e-4);
  }

  // The duty-less "stride * freq" shortcut the spec warns against must
  // NOT equal the true cap except at duty == 1 -- lock in that the real
  // formula, not the shortcut, is what's implemented.
  if (sp.duty < 0.999f) {
    const float shortcut = (2.f * halfSweepRad * limits.stanceRadiusMm) *
                            def.freqHz;  // == stride*freq, duty dropped
    CHECK(fabsf(shortcut - limits.vxMaxMmPerSec) > 0.5f);
  }
}

// --- 2. Non-finite inputs: rejected, never clamped into something
// plausible ----------------------------------------------------------
void testNonFiniteRejected(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);
  const float kNaN = NAN;
  const float kInf = INFINITY;

  GaitCommand base;
  base.bodyHeightMm = achievedHeight(sp.legMm);
  base.stepHeightMm = 0.1f * sp.legMm;
  base.vxMmPerSec = 10.f;
  base.omegaDegPerSec = 5.f;

  const int kFieldCount = 5;
  for (int field = 0; field < kFieldCount; ++field) {
    for (int poison = 0; poison < 2; ++poison) {
      Envelope env;
      GaitCommand bad = base;
      const float v = poison == 0 ? kNaN : kInf;
      switch (field) {
        case 0: bad.vxMmPerSec = v; break;
        case 1: bad.omegaDegPerSec = v; break;
        case 2: bad.bodyHeightMm = v; break;
        case 3: bad.stepHeightMm = v; break;
        case 4: bad.maxStrideMm = v; break;
      }
      const EnvelopeResult r =
          env.apply(bad, 1u, 0.f, ObstacleState::Clear, geo[0], def);
      CHECK(!r.accepted);
      CHECK(r.reason == RejectReason::Malformed);
      // "Never clamped into something plausible": the output must be
      // the safe all-zero default, not some clamped-looking finite
      // number derived from the NaN/Inf.
      CHECK(r.clamped.vxMmPerSec == 0.f);
      CHECK(r.clamped.omegaDegPerSec == 0.f);
      CHECK(r.clamped.bodyHeightMm == 0.f);
      CHECK(r.clamped.stepHeightMm == 0.f);
      CHECK(!r.wasClamped);

      // A rejected-as-Malformed command must not burn the seq slot --
      // the SAME seq must still be usable by a well-formed follow-up.
      const EnvelopeResult retry =
          env.apply(base, 1u, 1.f, ObstacleState::Clear, geo[0], def);
      CHECK(retry.accepted);
    }
  }
}

// --- 3. EStop latches and cannot be bypassed by any command sequence --
void testEstopLatchCannotBeBypassed(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);

  Envelope env;
  GaitCommand cmd;
  cmd.bodyHeightMm = achievedHeight(sp.legMm);
  cmd.stepHeightMm = 0.1f * sp.legMm;
  cmd.vxMmPerSec = 5.f;

  // Accepted before the latch.
  CHECK(env.apply(cmd, 1u, 0.f, ObstacleState::Clear, geo[0], def).accepted);

  env.triggerEstop();
  CHECK(env.estopped());

  // Every kind of command that might plausibly "look like" a reset
  // attempt: fresh seq, later timestamp, obstacle Clear, a Stop-shaped
  // (all-zero) command, an identical replay -- all rejected, every time.
  GaitCommand zero;
  const GaitCommand attempts[] = {cmd, zero};
  uint32_t seq = 2;
  float t = 1.f;
  for (int round = 0; round < 20; ++round) {
    for (unsigned a = 0; a < 2; ++a) {
      const EnvelopeResult r =
          env.apply(attempts[a], seq, t, ObstacleState::Clear, geo[0], def);
      CHECK(!r.accepted);
      CHECK(r.reason == RejectReason::Estopped);
      ++seq;
      t += 0.1f;
    }
  }
  CHECK(env.estopped());

  // The ONLY way out: an explicit clear, with a strictly newer seq than
  // anything accepted (or attempted) before.
  env.clearEstop();
  CHECK(!env.estopped());
  const EnvelopeResult after =
      env.apply(cmd, seq, t, ObstacleState::Clear, geo[0], def);
  CHECK(after.accepted);
}

// --- 4. Stale/replayed seq rejected; fresh accepted; wraparound sane --
void testSeqHandling(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);
  GaitCommand cmd;
  cmd.bodyHeightMm = achievedHeight(sp.legMm);

  Envelope env;
  CHECK(env.apply(cmd, 5u, 0.f, ObstacleState::Clear, geo[0], def).accepted);
  // Exact replay.
  CHECK(!env.apply(cmd, 5u, 1.f, ObstacleState::Clear, geo[0], def).accepted);
  // Reorder (older seq arriving late).
  CHECK(!env.apply(cmd, 3u, 2.f, ObstacleState::Clear, geo[0], def).accepted);
  const EnvelopeResult stale =
      env.apply(cmd, 3u, 2.f, ObstacleState::Clear, geo[0], def);
  CHECK(stale.reason == RejectReason::StaleSeq);
  // Fresh.
  CHECK(env.apply(cmd, 6u, 3.f, ObstacleState::Clear, geo[0], def).accepted);

  // Wraparound: lastSeq_ sits at UINT32_MAX-ish, next seq legitimately
  // wraps back around to a small number -- must still read as "newer".
  Envelope wrapEnv;
  CHECK(wrapEnv.apply(cmd, 0xFFFFFFFEu, 0.f, ObstacleState::Clear, geo[0], def)
            .accepted);
  CHECK(wrapEnv.apply(cmd, 0xFFFFFFFFu, 1.f, ObstacleState::Clear, geo[0], def)
            .accepted);
  CHECK(
      wrapEnv.apply(cmd, 0u, 2.f, ObstacleState::Clear, geo[0], def).accepted);
  CHECK(
      wrapEnv.apply(cmd, 1u, 3.f, ObstacleState::Clear, geo[0], def).accepted);
  // But a seq that "wrapped" the wrong way (an old one re-sent) is still
  // stale even across the wrap boundary.
  CHECK(!wrapEnv.apply(cmd, 0xFFFFFFFEu, 4.f, ObstacleState::Clear, geo[0], def)
             .accepted);
}

// --- 5. Rate limiting triggers and recovers ----------------------------
void testRateLimiting(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);
  GaitCommand cmd;
  cmd.bodyHeightMm = achievedHeight(sp.legMm);

  Envelope env;  // default config: kDefaultMinDriveIntervalSec
  const EnvelopeResult first =
      env.apply(cmd, 1u, 0.f, ObstacleState::Clear, geo[0], def);
  CHECK(first.accepted);

  // Arrives well inside the minimum interval: rejected.
  const EnvelopeResult tooSoon = env.apply(
      cmd, 2u, kDefaultMinDriveIntervalSec * 0.1f, ObstacleState::Clear,
      geo[0], def);
  CHECK(!tooSoon.accepted);
  CHECK(tooSoon.reason == RejectReason::RateLimited);

  // Recovers once enough time has passed since the last ACCEPTED
  // command (not since the rejected one).
  const EnvelopeResult recovered = env.apply(
      cmd, 3u, kDefaultMinDriveIntervalSec * 1.5f, ObstacleState::Clear,
      geo[0], def);
  CHECK(recovered.accepted);

  // A non-finite timestamp must fail safe, not bypass the limiter.
  const EnvelopeResult badClock = env.apply(
      cmd, 4u, NAN, ObstacleState::Clear, geo[0], def);
  CHECK(!badClock.accepted);
  CHECK(badClock.reason == RejectReason::RateLimited);
}

// --- 6. Obstacle seam is inert with Clear -------------------------------
void testObstacleSeamInert(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);
  GaitCommand cmd;
  cmd.bodyHeightMm = achievedHeight(sp.legMm);
  cmd.vxMmPerSec = 1e6f;  // even a wild request must never be rejected
  cmd.omegaDegPerSec = -1e6f;  // for an OBSTACLE reason -- clamping is
                                 // a separate concern from gating.

  Envelope env;
  for (uint32_t i = 0; i < 50; ++i) {
    const EnvelopeResult r = env.apply(cmd, i + 1u, float(i) * 10.f,
                                        ObstacleState::Clear, geo[0], def);
    CHECK(r.reason != RejectReason::Obstacle);
  }

  // The seam IS wired -- Blocked does reject -- which is what makes the
  // Clear-is-inert assertion above meaningful rather than vacuous. This
  // is NOT a claim that obstacle avoidance works: nothing in this
  // codebase can currently produce Blocked (see envelope.h).
  Envelope env2;
  const EnvelopeResult blocked =
      env2.apply(cmd, 1u, 0.f, ObstacleState::Blocked, geo[0], def);
  CHECK(!blocked.accepted);
  CHECK(blocked.reason == RejectReason::Obstacle);
}

// --- 7/8. A clamped command is always within the derived limits, and
// satisfies the constrained system (the gait) downstream ---------------
void testClampStaysWithinLimitsAndGaitAccepts(const SweepPoint& sp) {
  LegGeometry geo[kLegCount];
  buildLegs(sp.coxaMm, sp.legMm, geo);
  const GaitDef def = gaitWithDuty(sp.duty);

  const float wildVx[] = {1e6f, -1e6f, 0.f};
  const float wildOmega[] = {1e6f, -1e6f, 0.f};
  const float wildHeight[] = {-1e6f, 1e6f, achievedHeight(sp.legMm)};
  const float wildStep[] = {-1e6f, 1e6f, 0.f};

  uint32_t seq = 1;
  float t = 0.f;
  for (unsigned iv = 0; iv < 3; ++iv) {
    for (unsigned io = 0; io < 3; ++io) {
      for (unsigned ih = 0; ih < 3; ++ih) {
        for (unsigned is = 0; is < 3; ++is) {
          Envelope env;  // fresh, so rate limiting never interferes here
          GaitCommand req;
          req.vxMmPerSec = wildVx[iv];
          req.omegaDegPerSec = wildOmega[io];
          req.bodyHeightMm = wildHeight[ih];
          req.stepHeightMm = wildStep[is];
          req.maxStrideMm = 0.f;

          const EnvelopeResult r =
              env.apply(req, seq, t, ObstacleState::Clear, geo[0], def);
          ++seq;
          t += 1.f;
          CHECK(r.accepted);

          const EnvelopeLimits limits =
              computeLimits(geo[0], def, req.bodyHeightMm);
          // 7. Within the derived limits, by construction.
          CHECK(r.clamped.vxMmPerSec >= -limits.vxMaxMmPerSec - 1e-3f);
          CHECK(r.clamped.vxMmPerSec <= limits.vxMaxMmPerSec + 1e-3f);
          CHECK(r.clamped.omegaDegPerSec >= -limits.omegaMaxDegPerSec - 1e-3f);
          CHECK(r.clamped.omegaDegPerSec <= limits.omegaMaxDegPerSec + 1e-3f);
          CHECK(r.clamped.bodyHeightMm >= limits.bodyHeightMinMm - 1e-3f);
          CHECK(r.clamped.bodyHeightMm <= limits.bodyHeightMaxMm + 1e-3f);
          CHECK(r.clamped.stepHeightMm >= -1e-3f);
          CHECK(r.clamped.stepHeightMm <= limits.stepHeightMaxMm + 1e-3f);

          // 8. Feed the clamped output into the actual constrained
          // system: every emitted leg angle must be exactly reachable
          // (this is the gait's own invariant -- true by construction
          // whenever nothing upstream fed it a NaN or skipped the
          // envelope, which is precisely what's being checked here).
          GaitScheduler s;
          s.setGeometry(geo);
          s.setGait(def);
          s.setCommand(r.clamped);
          LegAngles ang[kLegCount];
          GaitReport rep;
          s.step(0.001f, 1.f, ang, rep);
          for (uint8_t i = 0; i < kLegCount; ++i) {
            const Vec3 p = forwardKinematics(geo[i], ang[i]);
            const IkResult ik = solveFootPosition(geo[i], p);
            CHECK(ik.reachable);
          }
        }
      }
    }
  }
}

}  // namespace

int main() {
  buildSweep();

  for (unsigned g = 0; g < kSweepCount; ++g) {
    const SweepPoint& sp = kSweep[g];
    const int before = g_fails;

    testDerivedCapsMatchGaitFormula(sp);
    testNonFiniteRejected(sp);
    testEstopLatchCannotBeBypassed(sp);
    testSeqHandling(sp);
    testRateLimiting(sp);
    testObstacleSeamInert(sp);
    testClampStaysWithinLimitsAndGaitAccepts(sp);

    if (g_fails != before) {
      printf("  ^ failures above at coxa=%.0f leg=%.0f duty=%.2f\n",
             double(sp.coxaMm), double(sp.legMm), double(sp.duty));
    }
  }

  if (g_fails == 0) {
    printf("test_envelope: all passed (%u geometries)\n", kSweepCount);
  }
  return g_fails ? 1 : 0;
}
