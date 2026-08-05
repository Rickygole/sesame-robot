// gait.h -- phase scheduler that turns a single body-velocity command
// into per-leg foot trajectories. Forward, backward, strafe, spin and
// arcing turns all come out of the SAME code path (v_leg = (vx,vy) +
// omega x r_hip per leg); there are no special-case turn functions.
#pragma once

#include "types.h"
#include "vec3.h"

namespace sesame {
namespace core {

struct GaitDef {
  const char* name;
  float phaseOffset[kLegCount];  // per-leg phase offset, indexed by Leg
  float dutyFactor;              // fraction of the cycle each leg spends in stance
  float freqHz;                  // cycle frequency at throttle == 1
};

// Crawl: 3 feet down at all times => statically stable. DEFAULT gait.
constexpr GaitDef kCrawl{
    "crawl", {0.00f, 0.50f, 0.25f, 0.75f}, 0.75f, 0.7f};
// Trot: diagonal leg pairs move together. Dynamically stable only.
constexpr GaitDef kTrot{"trot", {0.00f, 0.50f, 0.50f, 0.00f}, 0.50f, 1.4f};

struct GaitCommand {
  float vx = 0.f;             // mm/s, body-frame forward
  float vy = 0.f;             // mm/s, body-frame left
  float omega = 0.f;          // rad/s, +ccw viewed from above
  float bodyHeightMm = 0.f;   // nominal stance height (positive number, foot sits at -bodyHeightMm)
  float stepHeightMm = 0.f;   // swing foot clearance
};

// TODO(hardware): PLACEHOLDER footprint. No physical robot exists to
// measure the real hip layout yet; these must be verified against CAD
// before trusting them for anything beyond simulation. Body frame:
// x = forward, y = left, z = up (see geometry.h).
constexpr float kPlaceholderBodyHalfLengthMm = 40.f;
constexpr float kPlaceholderBodyHalfWidthMm = 35.f;

// Nominal (zero-velocity) foot anchor point for each leg, in the body
// frame, at zero height (z handled separately via bodyHeightMm).
Vec3 nominalFootAnchor(Leg leg);

class GaitScheduler {
 public:
  GaitScheduler();

  void setGait(const GaitDef& def);
  void setCommand(const GaitCommand& cmd);

  // Advances the gait clock by dt * freqHz * throttle (throttle in
  // [0,1], typically SlewResult::throttle -- this is what keeps the
  // gait coherent under power limiting instead of the pose distorting)
  // and writes each leg's foot position, BODY frame, into
  // outFeetBodyFrame[uint8_t(Leg)].
  void step(float dt, float throttle, Vec3 outFeetBodyFrame[kLegCount]);

  float phase() const { return phase_; }
  bool atCycleBoundary() const { return atCycleBoundary_; }
  const GaitDef& gait() const { return def_; }

  // Wrapped local phase in [0,1) for a single leg at the CURRENT
  // scheduler phase: wrap(phase() + gait().phaseOffset[leg]).
  float legPhase(Leg leg) const;
  // legPhase(leg) < gait().dutyFactor.
  bool legInStance(Leg leg) const;

 private:
  GaitDef def_;
  GaitCommand cmd_;
  float phase_;
  bool atCycleBoundary_;
};

}  // namespace core
}  // namespace sesame
