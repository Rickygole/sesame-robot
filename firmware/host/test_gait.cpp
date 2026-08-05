#include <cmath>
#include <cstdio>

#include "check.h"
#include "core/gait.h"
#include "core/trajectory.h"
#include "core/types.h"
#include "core/vec3.h"

int g_fails = 0;

using sesame::core::GaitCommand;
using sesame::core::GaitScheduler;
using sesame::core::kCrawl;
using sesame::core::kLegCount;
using sesame::core::kTrot;
using sesame::core::Leg;
using sesame::core::nominalFootAnchor;
using sesame::core::stanceDisplacement;
using sesame::core::swingTrajectory;
using sesame::core::Vec3;

static float wrap01(float x) { return x - std::floor(x); }

static void testPhaseWrapsCleanly() {
  GaitScheduler sched;
  sched.setGait(kCrawl);  // freqHz = 0.7
  Vec3 feet[kLegCount];

  const float dt = 0.05f;
  float expected = 0.f;
  for (int i = 0; i < 400; ++i) {
    sched.step(dt, 1.f, feet);
    expected = wrap01(expected + dt * kCrawl.freqHz);
    CHECK(sched.phase() >= 0.f);
    CHECK(sched.phase() < 1.f);
    CHECK_NEAR(sched.phase(), expected, 1e-3);
  }
}

static void testAtCycleBoundary() {
  GaitScheduler sched;
  sched.setGait(kCrawl);
  Vec3 feet[kLegCount];
  // freqHz=0.7 => one full cycle every ~1.4286s. Step exactly one cycle.
  const float period = 1.f / kCrawl.freqHz;
  sched.step(period * 0.5f, 1.f, feet);
  CHECK(!sched.atCycleBoundary());
  sched.step(period * 0.5f, 1.f, feet);  // now crosses phase == 1.0
  CHECK(sched.atCycleBoundary());
  CHECK_NEAR(sched.phase(), 0.0, 1e-4);
}

static void testLegPhaseOffsetsCorrect() {
  // freqHz == 1 makes dt == target phase directly from a phase-0 start.
  GaitScheduler sched;
  sesame::core::GaitDef unitFreqCrawl = kCrawl;
  unitFreqCrawl.freqHz = 1.f;
  sched.setGait(unitFreqCrawl);
  Vec3 feet[kLegCount];
  sched.step(0.3f, 1.f, feet);
  CHECK_NEAR(sched.phase(), 0.3, 1e-6);

  CHECK_NEAR(sched.legPhase(Leg::FR), wrap01(0.3f + 0.00f), 1e-5);
  CHECK_NEAR(sched.legPhase(Leg::RR), wrap01(0.3f + 0.50f), 1e-5);
  CHECK_NEAR(sched.legPhase(Leg::FL), wrap01(0.3f + 0.25f), 1e-5);
  CHECK_NEAR(sched.legPhase(Leg::RL), wrap01(0.3f + 0.75f), 1e-5);
}

static void testCrawlAlwaysAtLeastThreeStance() {
  GaitScheduler sched;
  sesame::core::GaitDef unitFreqCrawl = kCrawl;
  unitFreqCrawl.freqHz = 1.f;
  const int kSamples = 2000;
  for (int i = 0; i < kSamples; ++i) {
    GaitScheduler s;
    s.setGait(unitFreqCrawl);
    Vec3 feet[kLegCount];
    const float p = (float(i) + 0.5f) / float(kSamples);  // avoid exact boundaries
    s.step(p, 1.f, feet);

    int stanceCount = 0;
    for (uint8_t leg = 0; leg < kLegCount; ++leg) {
      if (s.legInStance(Leg(leg))) ++stanceCount;
    }
    CHECK(stanceCount >= 3);
  }
}

static void testFootTrajectoryC0AtStanceSwingBoundary() {
  const float strideLen = 40.f;
  const float stepHeight = 15.f;

  const float sStanceEnd = stanceDisplacement(strideLen, 1.f);
  const float sSwingStart = swingTrajectory(strideLen, stepHeight, 0.f).s;
  const float hSwingStart = swingTrajectory(strideLen, stepHeight, 0.f).h;
  CHECK_NEAR(sStanceEnd, sSwingStart, 1e-4);
  CHECK_NEAR(hSwingStart, 0.0, 1e-4);

  const float sSwingEnd = swingTrajectory(strideLen, stepHeight, 1.f).s;
  const float hSwingEnd = swingTrajectory(strideLen, stepHeight, 1.f).h;
  const float sStanceStart = stanceDisplacement(strideLen, 0.f);
  CHECK_NEAR(sSwingEnd, sStanceStart, 1e-4);
  CHECK_NEAR(hSwingEnd, 0.0, 1e-4);
}

static void testCycloidZeroHorizontalVelocityAtTouchdownAndLiftoff() {
  const float strideLen = 40.f;
  const float stepHeight = 15.f;
  const float du = 1e-4f;

  // Liftoff (u=0): forward difference slope should be ~0.
  const float sAt0 = swingTrajectory(strideLen, stepHeight, 0.f).s;
  const float sAtDu = swingTrajectory(strideLen, stepHeight, du).s;
  const float slopeAt0 = (sAtDu - sAt0) / du;
  CHECK_NEAR(slopeAt0, 0.0, 0.05 * strideLen);

  // Touchdown (u=1): backward difference slope should be ~0.
  const float sAt1 = swingTrajectory(strideLen, stepHeight, 1.f).s;
  const float sAt1mDu = swingTrajectory(strideLen, stepHeight, 1.f - du).s;
  const float slopeAt1 = (sAt1 - sAt1mDu) / du;
  CHECK_NEAR(slopeAt1, 0.0, 0.05 * strideLen);
}

static void testPureOmegaNoNetTranslation() {
  const float omega = 1.2f;  // rad/s
  sesame::core::GaitDef unitFreqCrawl = kCrawl;
  unitFreqCrawl.freqHz = 1.f;

  GaitCommand cmd;
  cmd.vx = 0.f;
  cmd.vy = 0.f;
  cmd.omega = omega;
  cmd.bodyHeightMm = 50.f;
  cmd.stepHeightMm = 10.f;

  Vec3 impliedVLeg[kLegCount];

  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    GaitScheduler sched;
    sched.setGait(unitFreqCrawl);
    sched.setCommand(cmd);

    // Land this leg mid-stance (localPhase == 0.375, well inside [0,0.75)).
    const float targetLocalPhase = 0.375f;
    const float startPhase =
        wrap01(targetLocalPhase - unitFreqCrawl.phaseOffset[i]);

    Vec3 feet[kLegCount];
    sched.step(startPhase, 1.f, feet);
    CHECK(sched.legInStance(leg));
    const Vec3 p0 = feet[i];

    const float dPhase = 1e-4f;
    sched.step(dPhase, 1.f, feet);  // freqHz==1, so dt==dPhase
    CHECK(sched.legInStance(leg));
    const Vec3 p1 = feet[i];

    const Vec3 measuredFootVel((p1.x - p0.x) / dPhase, (p1.y - p0.y) / dPhase, 0.f);
    // Stance foot moves opposite the body's velocity relative to ground.
    impliedVLeg[i] = Vec3(-measuredFootVel.x, -measuredFootVel.y, 0.f);

    // And it should match the closed-form omega x r_hip exactly.
    const Vec3 r = nominalFootAnchor(leg);
    const float expectedVx = -omega * r.y;
    const float expectedVy = omega * r.x;
    CHECK_NEAR(impliedVLeg[i].x, expectedVx, 0.05);
    CHECK_NEAR(impliedVLeg[i].y, expectedVy, 0.05);
  }

  // Symmetric footprint => sum(r_i) == 0 => sum(omega x r_i) == 0:
  // the four legs' implied body velocities cancel exactly, i.e. pure
  // rotation about the body center with no net translation.
  Vec3 sum(0.f, 0.f, 0.f);
  for (uint8_t i = 0; i < kLegCount; ++i) sum += impliedVLeg[i];
  CHECK_NEAR(sum.x, 0.0, 0.1);
  CHECK_NEAR(sum.y, 0.0, 0.1);
}

int main() {
  testPhaseWrapsCleanly();
  testAtCycleBoundary();
  testLegPhaseOffsetsCorrect();
  testCrawlAlwaysAtLeastThreeStance();
  testFootTrajectoryC0AtStanceSwingBoundary();
  testCycloidZeroHorizontalVelocityAtTouchdownAndLiftoff();
  testPureOmegaNoNetTranslation();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
