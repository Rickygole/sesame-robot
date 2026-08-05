// test_safety.cpp -- regression tests for the "no input may produce a
// non-finite or out-of-envelope servo command" invariant.
//
// Every test here corresponds to a bug that was found and proven in
// review. They are grouped by the layer they defend, because the whole
// point is defense in depth: each of these bugs was individually
// sufficient to drive a servo to an undefined pulse width.
#include <math.h>
#include <stdio.h>

#include "check.h"
#include "core/calibration.h"
#include "core/command.h"
#include "core/mathutil.h"
#include "core/pose.h"
#include "core/slew.h"

using namespace sesame::core;

int g_fails = 0;

namespace {

const float kNaN = NAN;
const float kInf = INFINITY;

// --- Layer 1: the clamp primitive -----------------------------------

void testClampfNaN() {
  // The original bug: `v<lo ? lo : (v>hi ? hi : v)` returns NaN for NaN,
  // because both comparisons are false.
  CHECK(clampf(kNaN, -10.f, 10.f, 0.f) == 0.f);
  CHECK(clampf(kNaN, -10.f, 10.f, 7.5f) == 7.5f);
  CHECK(!std::isnan(clampf(kNaN, -10.f, 10.f, 0.f)));
  // Infinities were always handled correctly; make sure that stayed true.
  CHECK(clampf(kInf, -10.f, 10.f, 0.f) == 10.f);
  CHECK(clampf(-kInf, -10.f, 10.f, 0.f) == -10.f);
  // Ordinary values unaffected.
  CHECK(clampf(3.f, -10.f, 10.f, 0.f) == 3.f);
  CHECK(clampf(-99.f, -10.f, 10.f, 0.f) == -10.f);
}

// --- Layer 2: the parser --------------------------------------------

void testParserRejectsNonFinite() {
  Command cmd;
  // The exact string proven to reach degToUs as NaN.
  CHECK(!parseCommand("drive nan 0 0 60 20", &cmd));
  // strtof accepts a whole family of spellings -- reject on VALUE, so
  // all of these must fail regardless of how they are written.
  const char* poison[] = {
      "drive NAN 0 0 60 20",       "drive nan(x) 0 0 60 20",
      "drive inf 0 0 60 20",       "drive +inf 0 0 60 20",
      "drive -inf 0 0 60 20",      "drive INFINITY 0 0 60 20",
      "drive infinity 0 0 60 20",  "drive 0 nan 0 60 20",
      "drive 0 0 nan 60 20",       "drive 0 0 0 nan 20",
      "drive 0 0 0 60 nan",
  };
  for (unsigned i = 0; i < sizeof(poison) / sizeof(poison[0]); ++i) {
    if (parseCommand(poison[i], &cmd)) {
      printf("FAIL parser accepted non-finite: \"%s\"\n", poison[i]);
      ++g_fails;
    }
  }
  // Legitimate commands must still parse.
  CHECK(parseCommand("drive 40 0 0 60 20", &cmd));
  CHECK(parseCommand("drive -40 0 30 60 20", &cmd));
  CHECK(parseCommand("drive 1e2 0 0 60 20", &cmd));
}

void testSetCalRejectsInsaneEnvelope() {
  Command cmd;
  // usMin >= usMax
  CHECK(!parseCommand("setcal 0 2000 2000 0 0 -60 90", &cmd));
  CHECK(!parseCommand("setcal 0 2500 1000 0 0 -60 90", &cmd));
  // Outside the absolute physical envelope.
  CHECK(!parseCommand("setcal 0 100 2929 0 0 -60 90", &cmd));
  CHECK(!parseCommand("setcal 0 732 32000 0 0 -60 90", &cmd));
  CHECK(!parseCommand("setcal 0 -32000 2929 0 0 -60 90", &cmd));
  // Inverted soft limits.
  CHECK(!parseCommand("setcal 0 732 2929 0 0 90 -60", &cmd));
  // The nominal MG90S calibration must still be accepted.
  CHECK(parseCommand("setcal 0 732 2929 0 0 -60 90", &cmd));
}

// --- Layer 3: the slew limiter --------------------------------------

void testSlewRejectsNonFinite() {
  JointPose cur;
  JointPose want;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    cur.deg[i] = 10.f;
    want.deg[i] = 20.f;
  }
  MotionLimits lim;

  // A single poisoned joint must not produce ANY non-finite output, and
  // must not be reported as an unthrottled (throttle==1) success.
  for (uint8_t bad = 0; bad < kJointCount; ++bad) {
    JointPose poisoned = want;
    poisoned.deg[bad] = kNaN;
    SlewResult r = applySlew(cur, poisoned, lim, 0.02f);
    for (uint8_t i = 0; i < kJointCount; ++i) {
      CHECK(isFiniteF(r.applied.deg[i]));
    }
    CHECK(r.throttle != 1.0f);
    CHECK(r.limited);
    // Fail-safe posture is "hold position".
    for (uint8_t i = 0; i < kJointCount; ++i) {
      CHECK_NEAR(r.applied.deg[i], cur.deg[i], 1e-6);
    }
  }

  // Infinity must be refused too -- clamping it to the per-joint rate
  // would silently turn "garbage" into "full-speed move".
  JointPose infPose = want;
  infPose.deg[2] = kInf;
  SlewResult ri = applySlew(cur, infPose, lim, 0.02f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK(isFiniteF(ri.applied.deg[i]));
    CHECK_NEAR(ri.applied.deg[i], cur.deg[i], 1e-6);
  }

  // Degenerate dt / limits must also hold position rather than divide or
  // extrapolate.
  const float badDt[] = {0.f, -0.02f, kNaN, kInf};
  for (unsigned i = 0; i < sizeof(badDt) / sizeof(badDt[0]); ++i) {
    SlewResult r = applySlew(cur, want, lim, badDt[i]);
    for (uint8_t j = 0; j < kJointCount; ++j) {
      CHECK(isFiniteF(r.applied.deg[j]));
      CHECK_NEAR(r.applied.deg[j], cur.deg[j], 1e-6);
    }
  }

  MotionLimits badLim;
  badLim.maxDegPerSec = kNaN;
  SlewResult rl = applySlew(cur, want, badLim, 0.02f);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK(isFiniteF(rl.applied.deg[i]));
  }

  // And the normal path must be untouched by all this guarding.
  //
  // Note the delta has to be small enough to fit the GLOBAL budget, not
  // just the per-joint rate: 8 joints x 4.0 deg/tick = 32 deg/tick would
  // blow the 480 deg/s * 0.02 s = 9.6 deg/tick budget and legitimately
  // throttle to 0.3. Use 1.0 deg/joint (8.0 total) so nothing is limited.
  JointPose small;
  for (uint8_t i = 0; i < kJointCount; ++i) {
    small.deg[i] = cur.deg[i] + 1.0f;
  }
  SlewResult ok = applySlew(cur, small, lim, 0.02f);
  CHECK(ok.throttle == 1.0f);
  CHECK(!ok.limited);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    CHECK_NEAR(ok.applied.deg[i], cur.deg[i] + 1.0f, 1e-5);
  }
}

// --- Layer 4: the final conversion to a pulse width ------------------

void testDegToUsAlwaysInEnvelope() {
  JointCal cal;  // defaults: 732..2929, trim 0, no invert, -60..90
  const float poison[] = {kNaN,  kInf,  -kInf, 1e30f, -1e30f, 0.f,
                          1e-30f, 400.f, -400.f, 89.9f, -59.9f};
  for (unsigned i = 0; i < sizeof(poison) / sizeof(poison[0]); ++i) {
    const int32_t us = degToUs(cal, poison[i]);
    if (us < cal.usMin || us > cal.usMax) {
      printf("FAIL degToUs(%g) = %d, outside [%d,%d]\n", double(poison[i]),
             int(us), int(cal.usMin), int(cal.usMax));
      ++g_fails;
    }
    CHECK(us >= kAbsServoUsMin && us <= kAbsServoUsMax);
  }

  // A corrupt calibration must not be able to widen the envelope.
  JointCal bad;
  bad.usMin = -30000;
  bad.usMax = 30000;
  for (unsigned i = 0; i < sizeof(poison) / sizeof(poison[0]); ++i) {
    const int32_t us = degToUs(bad, poison[i]);
    CHECK(us >= kAbsServoUsMin && us <= kAbsServoUsMax);
  }

  // Degenerate span must not divide-by-zero or escape the envelope.
  JointCal degen;
  degen.usMin = 1500;
  degen.usMax = 1500;
  const int32_t d = degToUs(degen, 0.f);
  CHECK(d >= kAbsServoUsMin && d <= kAbsServoUsMax);

  // NaN in the calibration fields themselves.
  JointCal nanCal;
  nanCal.trimDeg = kNaN;
  CHECK(degToUs(nanCal, 0.f) >= kAbsServoUsMin);
  CHECK(degToUs(nanCal, 0.f) <= kAbsServoUsMax);
  JointCal nanLimits;
  nanLimits.minDeg = kNaN;
  nanLimits.maxDeg = kNaN;
  const int32_t nl = degToUs(nanLimits, 30.f);
  CHECK(nl >= nanLimits.usMin && nl <= nanLimits.usMax);
}

void testUsToDegDegenerateSpan() {
  JointCal degen;
  degen.usMin = 1500;
  degen.usMax = 1500;
  CHECK(isFiniteF(usToDeg(degen, 1500)));
  CHECK(isFiniteF(usToDeg(degen, 2000)));
}

// --- Layer 5: calibration blob validation ---------------------------

void testIsValidChecksVersion() {
  Calibration cal;
  cal.crc = cal.computeCrc();
  CHECK(cal.isValid());

  // A blob from a different firmware version, with a CRC that correctly
  // matches its own contents, must still be rejected.
  Calibration future;
  future.version = 999u;
  future.crc = future.computeCrc();
  CHECK(!future.isValid());

  // Wrong magic still rejected.
  Calibration wrongMagic;
  wrongMagic.magic = 0xDEADBEEFu;
  wrongMagic.crc = wrongMagic.computeCrc();
  CHECK(!wrongMagic.isValid());

  // Erased flash patterns must not validate.
  Calibration erased;
  erased.magic = 0xFFFFFFFFu;
  erased.version = 0xFFFFFFFFu;
  erased.crc = 0xFFFFFFFFu;
  CHECK(!erased.isValid());
  Calibration zeroed;
  zeroed.magic = 0u;
  zeroed.version = 0u;
  zeroed.crc = 0u;
  CHECK(!zeroed.isValid());
}

// --- End to end ------------------------------------------------------

void testNoCommandCanEscapeEnvelope() {
  // The original kill chain, end to end: a malformed command string must
  // not be able to produce an out-of-envelope pulse width by any route.
  const char* lines[] = {
      "drive nan 0 0 60 20",   "drive inf 0 0 60 20",
      "drive 1e30 0 0 60 20",  "drive -1e30 0 0 60 20",
      "setcal 0 0 32000 0 0 -60 90",
  };
  JointCal cal;
  for (unsigned i = 0; i < sizeof(lines) / sizeof(lines[0]); ++i) {
    Command cmd;
    if (!parseCommand(lines[i], &cmd)) {
      continue;  // rejected at the parser -- the preferred outcome
    }
    // If it parsed, whatever it commands must still be safe.
    JointPose cur;
    JointPose want;
    for (uint8_t j = 0; j < kJointCount; ++j) {
      cur.deg[j] = 0.f;
      want.deg[j] = cmd.payload.drive.vx;
    }
    MotionLimits lim;
    SlewResult r = applySlew(cur, want, lim, 0.02f);
    for (uint8_t j = 0; j < kJointCount; ++j) {
      CHECK(isFiniteF(r.applied.deg[j]));
      const int32_t us = degToUs(cal, r.applied.deg[j]);
      CHECK(us >= kAbsServoUsMin && us <= kAbsServoUsMax);
    }
  }
}

}  // namespace

int main() {
  testClampfNaN();
  testParserRejectsNonFinite();
  testSetCalRejectsInsaneEnvelope();
  testSlewRejectsNonFinite();
  testDegToUsAlwaysInEnvelope();
  testUsToDegDegenerateSpan();
  testIsValidChecksVersion();
  testNoCommandCanEscapeEnvelope();
  if (g_fails == 0) {
    printf("test_safety: all passed\n");
  }
  return g_fails ? 1 : 0;
}
