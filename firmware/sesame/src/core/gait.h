// gait.h -- phase scheduler that turns a single body-velocity command into
// per-leg SERVO ANGLES, planned directly in the two coordinates a 2-DOF
// leg actually has (hip yaw theta, elevation phi) rather than in
// Cartesian space and then projected onto the reachable surface.
//
// Key fact this whole file leans on: in a leg's hip frame, both foot
// height and horizontal reach are functions of phi ALONE --
//   z(phi) = -legMm * sin(phi)
//   r(phi) =  coxaMm + legMm * cos(phi)
// -- so sweeping theta at CONSTANT phi traces a circle, every point of
// which is exactly reachable and at exactly the same height. Stance is
// therefore planned as "sweep theta at the stance phi"; swing lifts phi
// to a shallower angle and back down while continuing to sweep theta.
// Nothing here ever computes an (x,y,z) foot target and asks whether the
// leg can reach it -- it is unreachable BY CONSTRUCTION, never.
//
// Forward, backward, spin and arcing turns all come out of the SAME code
// path (v_leg = (vx,0) + omega x r_hip per leg); there are no special-case
// turn functions. There is deliberately no vy/strafe -- see GaitCommand.
#pragma once

#include "geometry.h"
#include "types.h"

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

// Mechanical cap on the half-sweep (theta excursion either side of
// neutral). This is a servo/mechanism limit, not a knob -- it exists so a
// command asking for an enormous stride does not swing a leg past a
// sane range. Expressed in degrees so it reads directly; gait.cpp
// converts to radians.
constexpr float kMaxHalfSweepDeg = 25.f;

struct GaitCommand {
  float vxMmPerSec = 0.f;        // +forward, body-frame
  float omegaDegPerSec = 0.f;    // +ccw viewed from above (yaw left)
  float bodyHeightMm = 0.f;      // nominal stance height (positive, foot
                                  // sits below the hip by this much)
  float stepHeightMm = 0.f;      // swing foot clearance above stance
  float maxStrideMm = 0.f;       // optional extra cap on peak-to-peak
                                  // stride length, mm; <= 0 means "no
                                  // extra cap beyond kMaxHalfSweepDeg"
  // NOTE: there is deliberately no vy/strafe field. A 2-DOF leg's only
  // achievable horizontal foot motion at neutral stance is TANGENTIAL to
  // its hip's stance circle; lateral (radial) foot velocity is achievable
  // only off-neutral, with authority that is sin(dTheta)-weighted and
  // FLIPS SIGN across the stride -- that is a phase-locked wobble, not a
  // strafe, so it is cut rather than degraded. See gait.cpp for the
  // velocity projection that reports this as scrub for omega commands.
};

// Everything ACHIEVED this tick, as opposed to what was asked for --
// this is what makes it possible to tell a real knob from a lie. Several
// fields are single scalars even though the underlying quantity is
// computed per leg (e.g. bodyHeightMm, stanceRadiusMm): on any real
// build all four legs share the same link lengths (LegGeometry.coxaMm /
// legMm), so these are identical across legs and reporting one value is
// exact, not an approximation. maxScrubMmPerSec and
// maxLateralDeviationMm are explicitly the WORST case across legs,
// because those genuinely differ leg-to-leg once omega != 0.
struct GaitReport {
  float bodyHeightMm = 0.f;          // ACHIEVED (clamped to [0.15,0.95]*legMm)
  float stepHeightMm = 0.f;          // ACHIEVED (clamped to [0,bodyHeightMm])
  float strideMm = 0.f;              // ACHIEVED peak-to-peak stride arc length
  float achievedVxMmPerSec = 0.f;    // vxMmPerSec * throttle
  float stanceRadiusMm = 0.f;        // DEPENDENT readout, not a knob: coxaMm + legMm*cos(phiSt)
  float maxScrubMmPerSec = 0.f;      // worst |radial foot velocity| across legs -- genuinely unachievable
  float maxLateralDeviationMm = 0.f; // worst rSt*(1-cos(halfSweep)) across legs -- the stride's arc bow
  bool clamped = false;              // true if any per-tick clamp bound (height, step height, or sweep)
};

class GaitScheduler {
 public:
  GaitScheduler();

  void setGait(const GaitDef& def);
  void setCommand(const GaitCommand& cmd);

  // Per-leg rigid geometry (link lengths, mount pose, neutral yaw). MUST
  // be called at least once before step() -- step() plans directly on
  // each leg's own reachable circle, so it needs the real geometry, not
  // a gait-local placeholder. Safe to call again any time (e.g. after a
  // calibration update); geometry is copied, not referenced.
  void setGeometry(const LegGeometry geo[kLegCount]);

  // Advances the gait clock by dt * freqHz * throttle (throttle in
  // [0,1], typically SlewResult::throttle -- this is what keeps the
  // gait coherent under power limiting instead of the pose distorting)
  // and writes each leg's ANATOMICAL servo angles into
  // outAngles[uint8_t(Leg)], plus a report of what was actually
  // achieved this tick into outReport.
  void step(float dt, float throttle, LegAngles outAngles[kLegCount],
            GaitReport& outReport);

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
  LegGeometry geo_[kLegCount] = {};
  float phase_;
  bool atCycleBoundary_;
};

}  // namespace core
}  // namespace sesame
