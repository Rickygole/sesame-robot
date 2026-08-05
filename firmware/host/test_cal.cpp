#include <cstdio>
#include <cstring>

#include "check.h"
#include "core/calibration.h"

int g_fails = 0;

using sesame::core::Calibration;
using sesame::core::degToUs;
using sesame::core::JointCal;
using sesame::core::usToDeg;

static void testDegToUsMonotone() {
  JointCal cal;
  int32_t prev = degToUs(cal, cal.minDeg);
  for (float d = cal.minDeg + 1.f; d <= cal.maxDeg; d += 1.f) {
    const int32_t us = degToUs(cal, d);
    CHECK(us >= prev);
    prev = us;
  }
}

static void testDegToUsClampsAtSoftLimits() {
  JointCal cal;
  const int32_t atMin = degToUs(cal, cal.minDeg);
  const int32_t belowMin = degToUs(cal, cal.minDeg - 100.f);
  CHECK(belowMin == atMin);

  const int32_t atMax = degToUs(cal, cal.maxDeg);
  const int32_t aboveMax = degToUs(cal, cal.maxDeg + 100.f);
  CHECK(aboveMax == atMax);
}

static void testInvertIsInvolution() {
  // Round trip degToUs -> usToDeg with invert set must recover the
  // original degrees (within pulse-width quantization), for both
  // invert settings and with a nonzero trim, as long as the value is
  // safely inside the soft limits (so the non-invertible clamp step
  // never engages).
  JointCal cal;
  cal.trimDeg = 3.f;

  // Range chosen so that neither trim nor invert can push the
  // intermediate value outside [minDeg,maxDeg] and trigger the
  // (non-invertible) soft-limit clamp for either invert setting.
  for (int inv = 0; inv < 2; ++inv) {
    cal.invert = inv != 0;
    for (float d = -20.f; d <= 20.001f; d += 5.f) {
      const int32_t us = degToUs(cal, d);
      const float back = usToDeg(cal, us);
      // ~0.041 deg per us of quantization noise at these usMin/usMax.
      CHECK_NEAR(back, d, 0.1);
    }
  }
}

static void testInvertMirrorsAboutZero() {
  JointCal cal;
  cal.invert = true;
  cal.minDeg = -90.f;
  cal.maxDeg = 90.f;  // wide enough that clamp never engages here
  const int32_t usPos = degToUs(cal, 20.f);
  cal.invert = false;
  const int32_t usNeg = degToUs(cal, -20.f);
  CHECK(usPos == usNeg);
}

static void testCrcDetectsSingleBitCorruption() {
  Calibration cal;
  cal.crc = cal.computeCrc();
  CHECK(cal.isValid());

  // Flip one bit in the middle of the joint array.
  uint8_t* raw = reinterpret_cast<uint8_t*>(&cal.joint[3]);
  raw[0] ^= 0x01;
  CHECK(!cal.isValid());
  CHECK(cal.crc != cal.computeCrc());
}

static void testCrcStableAcrossRecompute() {
  Calibration a;
  Calibration b;
  a.joint[0].trimDeg = 2.5f;
  b.joint[0].trimDeg = 2.5f;
  CHECK(a.computeCrc() == b.computeCrc());

  b.joint[0].trimDeg = 2.6f;
  CHECK(a.computeCrc() != b.computeCrc());
}

int main() {
  testDegToUsMonotone();
  testDegToUsClampsAtSoftLimits();
  testInvertIsInvolution();
  testInvertMirrorsAboutZero();
  testCrcDetectsSingleBitCorruption();
  testCrcStableAcrossRecompute();
  if (g_fails) {
    std::fprintf(stderr, "%d failure(s)\n", g_fails);
  }
  return g_fails ? 1 : 0;
}
