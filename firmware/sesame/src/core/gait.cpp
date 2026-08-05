#include "gait.h"

#include <cmath>

#include "ik.h"
#include "mathutil.h"
#include "vec3.h"

namespace sesame {
namespace core {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float degToRad(float deg) { return deg * (kPi / 180.f); }
inline float radToDeg(float rad) { return rad * (180.f / kPi); }
inline float wrap01(float x) { return x - floorf(x); }

// zHat x r for a planar r (z component ignored/irrelevant): a 90 degree
// CCW rotate in the XY plane. Used both for omega x (foot - com) and to
// turn a radial unit vector into its tangential partner.
inline Vec3 crossZ(const Vec3& r) { return Vec3(-r.y, r.x, 0.f); }

inline float dotXY(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y; }
}  // namespace

GaitScheduler::GaitScheduler()
    : def_(kCrawl), cmd_(), phase_(0.f), atCycleBoundary_(false) {}

void GaitScheduler::setGait(const GaitDef& def) { def_ = def; }

void GaitScheduler::setCommand(const GaitCommand& cmd) { cmd_ = cmd; }

void GaitScheduler::setGeometry(const LegGeometry geo[kLegCount]) {
  for (uint8_t i = 0; i < kLegCount; ++i) {
    geo_[i] = geo[i];
  }
}

float GaitScheduler::legPhase(Leg leg) const {
  return wrap01(phase_ + def_.phaseOffset[uint8_t(leg)]);
}

bool GaitScheduler::legInStance(Leg leg) const {
  return legPhase(leg) < def_.dutyFactor;
}

void GaitScheduler::step(float dt, float throttle,
                          LegAngles outAngles[kLegCount],
                          GaitReport& outReport) {
  const float advanced = phase_ + dt * def_.freqHz * throttle;
  atCycleBoundary_ = advanced >= 1.f;
  phase_ = wrap01(advanced);

  const float omegaRad = degToRad(cmd_.omegaDegPerSec);
  const Vec3 bodyVel(cmd_.vxMmPerSec, 0.f, 0.f);
  const float maxHalfSweepRad = degToRad(kMaxHalfSweepDeg);

  bool clamped = false;
  float maxScrub = 0.f;
  float maxLateralDeviation = 0.f;

  // Single-scalar report fields: taken from leg 0. Exact (not merely
  // representative) whenever all four legs share the same link lengths,
  // which is the only geometry a real robot has -- see GaitReport.
  float reportBodyHeight = 0.f;
  float reportStepHeight = 0.f;
  float reportStride = 0.f;
  float reportStanceRadius = 0.f;

  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    const LegGeometry& geo = geo_[i];

    // ---- Per leg, per tick: the only two asinf call SITES in this
    // whole design. Both bounds are FRACTIONS of this leg's own length,
    // never a literal mm constant, so this is correct for any geometry.
    const float loH = 0.15f * geo.legMm;
    const float hiH = 0.95f * geo.legMm;
    const float h = clampf(cmd_.bodyHeightMm, loH, hiH, 0.5f * (loH + hiH));
    if (h != cmd_.bodyHeightMm) clamped = true;
    const float phiSt = asinf(clampf(h / geo.legMm, -1.f, 1.f, 0.f));
    const float rSt = geo.coxaMm + geo.legMm * cosf(phiSt);

    const float lift = clampf(cmd_.stepHeightMm, 0.f, h, 0.f);
    if (lift != cmd_.stepHeightMm) clamped = true;
    const float phiApex = asinf(clampf((h - lift) / geo.legMm, -1.f, 1.f, 0.f));

    // ---- Velocity projection onto this leg's stance circle.
    const float thetaN = degToRad(geo.neutralYawDeg);
    const Vec3 nHatHip = radialUnit(thetaN);
    // Hip-frame direction -> body-frame direction: mirror y by
    // lateralSign, same convention as footHipFrameToBody (no translation
    // for a direction, only for a point).
    const Vec3 nHat(nHatHip.x, geo.lateralSign * nHatHip.y, 0.f);
    const Vec3 tHat = crossZ(nHat);

    const Vec3 footNeutral = geo.hipInBody + rSt * nHat;

    // vfoot = -( (vx,0) + omega x (foot - com) ), com == body origin.
    const Vec3 vfoot = (bodyVel + omegaRad * crossZ(footNeutral)) * -1.f;
    const float vt = dotXY(vfoot, tHat);  // achievable (tangential)
    const float vr = dotXY(vfoot, nHat);  // NOT achievable -- scrub
    const float scrub = fabsf(vr);
    if (scrub > maxScrub) maxScrub = scrub;

    float sweepCapRad = maxHalfSweepRad;
    if (cmd_.maxStrideMm > 0.f && rSt > 1e-6f) {
      const float strideCapRad = cmd_.maxStrideMm / (2.f * rSt);
      if (strideCapRad < sweepCapRad) sweepCapRad = strideCapRad;
    }
    const float rawA = (def_.freqHz > 1e-6f && rSt > 1e-6f)
                            ? fabsf(vt) * def_.dutyFactor / (2.f * def_.freqHz * rSt)
                            : 0.f;
    const float A = clampf(rawA, 0.f, sweepCapRad, 0.f);
    if (A < rawA) clamped = true;
    const float sign = (vt < 0.f) ? -1.f : 1.f;

    const float lateralDeviation = rSt * (1.f - cosf(A));
    if (lateralDeviation > maxLateralDeviation) maxLateralDeviation = lateralDeviation;

    // ---- Phase -> (theta, phi). Stance sweeps theta at CONSTANT phiSt
    // (zero body bob, exact reach); swing is a C1 Hermite in theta and a
    // sin^2 bump in phi, chosen so BOTH curves match the stance tangent
    // at liftoff and touchdown -- that is what keeps the whole cycle,
    // including the phase wrap, free of any slope discontinuity the
    // slew limiter could see as a spike.
    const float p = legPhase(leg);
    float dTh;
    float phi;
    if (p < def_.dutyFactor) {
      const float u = (def_.dutyFactor > 1e-6f) ? (p / def_.dutyFactor) : 0.f;
      dTh = sign * A * (1.f - 2.f * u);
      phi = phiSt;
    } else {
      const float swingSpan = 1.f - def_.dutyFactor;
      const float u = (swingSpan > 1e-6f) ? (p - def_.dutyFactor) / swingSpan : 0.f;
      const float k = (def_.dutyFactor > 1e-6f)
                           ? 2.f * A * swingSpan / def_.dutyFactor
                           : 0.f;
      const float h00 = 2.f * u * u * u - 3.f * u * u + 1.f;
      const float h10 = u * u * u - 2.f * u * u + u;
      const float h01 = -2.f * u * u * u + 3.f * u * u;
      const float h11 = u * u * u - u * u;
      dTh = sign * (h00 * (-A) + h01 * A - k * (h10 + h11));
      const float s = sinf(kPi * u);
      phi = phiSt + (phiApex - phiSt) * (s * s);
    }

    const float yawDeg = geo.neutralYawDeg + radToDeg(dTh);
    outAngles[i] = anglesFromYawPitch(geo, yawDeg, phi);

    if (i == 0) {
      reportBodyHeight = h;
      reportStepHeight = lift;
      reportStride = 2.f * A * rSt;
      reportStanceRadius = rSt;
    }
  }

  outReport.bodyHeightMm = reportBodyHeight;
  outReport.stepHeightMm = reportStepHeight;
  outReport.strideMm = reportStride;
  outReport.achievedVxMmPerSec = cmd_.vxMmPerSec * throttle;
  outReport.stanceRadiusMm = reportStanceRadius;
  outReport.maxScrubMmPerSec = maxScrub;
  outReport.maxLateralDeviationMm = maxLateralDeviation;
  outReport.clamped = clamped;
}

}  // namespace core
}  // namespace sesame
