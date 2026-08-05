#include "gait.h"

#include <cmath>

#include "trajectory.h"

namespace sesame {
namespace core {

Vec3 nominalFootAnchor(Leg leg) {
  const float hx = kPlaceholderBodyHalfLengthMm;
  const float hy = kPlaceholderBodyHalfWidthMm;
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

GaitScheduler::GaitScheduler()
    : def_(kCrawl), cmd_(), phase_(0.f), atCycleBoundary_(false) {}

void GaitScheduler::setGait(const GaitDef& def) { def_ = def; }

void GaitScheduler::setCommand(const GaitCommand& cmd) { cmd_ = cmd; }

float GaitScheduler::legPhase(Leg leg) const {
  float lp = phase_ + def_.phaseOffset[uint8_t(leg)];
  lp -= std::floor(lp);
  return lp;
}

bool GaitScheduler::legInStance(Leg leg) const {
  return legPhase(leg) < def_.dutyFactor;
}

void GaitScheduler::step(float dt, float throttle,
                          Vec3 outFeetBodyFrame[kLegCount]) {
  const float advanced = phase_ + dt * def_.freqHz * throttle;
  atCycleBoundary_ = advanced >= 1.f;
  phase_ = advanced - std::floor(advanced);

  for (uint8_t i = 0; i < kLegCount; ++i) {
    const Leg leg = Leg(i);
    const Vec3 anchor = nominalFootAnchor(leg);
    const float rx = anchor.x;
    const float ry = anchor.y;

    // v_leg = (vx,vy) + omega x r_hip. This ONE formula is forward,
    // backward, strafe, spin AND arcing turns -- no special cases.
    const float vlx = cmd_.vx - cmd_.omega * ry;
    const float vly = cmd_.vy + cmd_.omega * rx;
    const float speed = std::sqrt(vlx * vlx + vly * vly);

    float dirX = 1.f;
    float dirY = 0.f;
    if (speed > 1e-6f) {
      dirX = vlx / speed;
      dirY = vly / speed;
    }

    const float strideLen =
        (def_.freqHz > 1e-6f) ? (speed * def_.dutyFactor / def_.freqHz) : 0.f;

    const float localPhase = legPhase(leg);

    float s;
    float h;
    if (localPhase < def_.dutyFactor) {
      const float u = localPhase / def_.dutyFactor;
      s = stanceDisplacement(strideLen, u);
      h = 0.f;
    } else {
      const float u = (localPhase - def_.dutyFactor) / (1.f - def_.dutyFactor);
      const SwingSample sample = swingTrajectory(strideLen, cmd_.stepHeightMm, u);
      s = sample.s;
      h = sample.h;
    }

    outFeetBodyFrame[i] =
        Vec3(anchor.x + dirX * s, anchor.y + dirY * s, -cmd_.bodyHeightMm + h);
  }
}

}  // namespace core
}  // namespace sesame
