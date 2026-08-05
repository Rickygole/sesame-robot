// envelope.h -- the layer that decides whether a command is ALLOWED and
// clamps it to what the machine can physically do, BEFORE the gait ever
// sees it. slew.h then limits the RATE at which an already-clamped
// target is approached. Two different jobs, both mandatory, in that
// order: nothing may bypass either. See core/README.md for the purity
// contract this file must obey (no Arduino, no wall clock, no heap).
#pragma once

#include <stdint.h>

#include "gait.h"
#include "geometry.h"

namespace sesame {
namespace core {

// ---- Derived physical limits ------------------------------------------
//
// Every field here is COMPUTED from LegGeometry + GaitDef, never typed
// in as a constant. A hardcoded plausible-looking cap would be silently
// unreachable (too conservative) or silently unsafe (too permissive)
// the instant geometry changes -- that is exactly the bug class that
// made the original gait's knobs meaningless (see gait.h/.cpp's
// commentary on the same failure mode), so it must not recur here.
struct EnvelopeLimits {
  float vxMaxMmPerSec;        // derived forward-speed cap, see computeLimits()
  float omegaMaxDegPerSec;    // derived yaw-rate cap, see computeLimits()
  float bodyHeightMinMm;      // 0.15 * legMm, matches gait.cpp's own clamp
  float bodyHeightMaxMm;      // 0.95 * legMm, matches gait.cpp's own clamp
  float stepHeightMaxMm;      // ACHIEVED bodyHeightMm (post height-clamp);
                               // gait.cpp clamps stepHeightMm to [0, h], so
                               // this is that same bound, not the raw request
  float stanceRadiusMm;       // dependent readout, not a knob: coxaMm +
                               // legMm*cos(phiSt) at the achieved height
};

// Derives EnvelopeLimits for `def` run on `geo`, at the given requested
// body height (internally clamped to [0.15,0.95]*legMm first, exactly
// as gait.cpp itself does, so vxMax/omegaMax are computed at the height
// the gait will actually achieve, not the raw, possibly out-of-range,
// request).
//
// --- Derivation (mirrors the ONLY other place this math lives, the
// per-leg stance loop in gait.cpp) ---
//
// The gait computes each leg's half-sweep as
//   A = |vt| * duty / (2 * freqHz * rSt),   rSt = coxaMm + legMm*cos(phiSt)
// where phiSt = asin(bodyHeightMm / legMm) and vt is the tangential
// component of the requested foot velocity (== |vx| when omega == 0 --
// see gait.cpp's velocity projection). Inverting that at the mechanical
// sweep cap A == kMaxHalfSweepDeg gives the true achievable speed cap:
//
//   vxMax = radians(kMaxHalfSweepDeg) * 2 * freqHz * rSt / duty
//
// This is NOT `stride * freq` (that drops the duty term and is only
// correct at duty==1); on the placeholder geometry the duty-term-free
// formula gives ~44 mm/s against the true ~59.2 mm/s cap, a 25%
// understatement that would leave a real knob silently unreachable.
//
// omegaMaxDegPerSec is the same inversion, but for a pure-yaw command
// (vx == 0): a leg spinning about the stance circle sees tangential
// foot speed omega * rSt, so substituting |vt| = omegaRad * rSt into the
// same equation and solving for omegaRad cancels rSt entirely:
//   omegaMax = radians(kMaxHalfSweepDeg) * 2 * freqHz / duty
// NOTE this treats the leg's own stance-circle radius as the moment arm
// and, like the rest of this single-LegGeometry API, does not account
// for hipInBody (the leg's offset from the body's yaw axis) -- gait.cpp
// DOES include that offset per leg (see its `footNeutral` /
// omega-cross-r term), so for a leg mounted far from the body center
// this cap can be a slight over- or under-estimate of that leg's true
// scrub-free omega ceiling. gait.cpp's own per-tick sweep clamp
// (`A = clampf(rawA, 0, sweepCapRad, 0)`) is the final backstop
// regardless, so this approximation never lets an unreachable command
// through unclamped -- it only affects how tight the *reported* cap is.
//
// Also note vxMaxMmPerSec and omegaMaxDegPerSec are independent
// SINGLE-AXIS caps (each derived assuming the other is zero). Clamping
// vx and omega separately to their own caps does not guarantee the
// COMBINED (vx, omega) pair stays under kMaxHalfSweepDeg for every leg
// -- the two can add constructively for one leg while cancelling for
// another (see gait.cpp's per-leg vt projection). gait.cpp's own
// per-tick sweep clamp is what makes a combined command safe regardless
// (GaitReport::clamped reports when it had to); Envelope's clamp is a
// best-effort pre-filter, not a substitute for that final clamp.
EnvelopeLimits computeLimits(const LegGeometry& geo, const GaitDef& def,
                              float bodyHeightMm);

// ---- Obstacle seam: SHIPPED DARK, NOT IMPLEMENTED ----------------------
//
// There is no ultrasonic (or any other) proximity sensor in this
// firmware. ObstacleState exists so the CALL SHAPE of obstacle-aware
// gating is correct end to end -- Envelope::apply() takes one, and
// rejects with RejectReason::Obstacle when it is not Clear -- but
// nothing in this codebase can currently produce anything other than
// Clear. Do not read the presence of this enum, or of
// RejectReason::Obstacle, as evidence that obstacle avoidance works. It
// does not. It is wired as an inert pass-through and tested ONLY as an
// inert pass-through (see firmware/host/test_envelope.cpp). When a real
// sensor is added, whatever reads it should compute an ObstacleState and
// pass it into apply() each tick; nothing else in this file needs to
// change.
enum class ObstacleState : uint8_t {
  Clear = 0,
  Blocked = 1,
};

// Why a command was not accepted as-is. None means it was accepted
// (possibly clamped -- see EnvelopeResult::wasClamped).
enum class RejectReason : uint8_t {
  None = 0,
  Estopped = 1,   // latched E-stop; see Envelope::triggerEstop()/clearEstop()
  Obstacle = 2,   // see ObstacleState above -- currently unreachable, stub only
  RateLimited = 3,  // Drive commands arriving faster than kDefaultMinDriveIntervalSec
  StaleSeq = 4,     // seq not newer than the last accepted seq (replay/reorder)
  Malformed = 5,    // NaN/Inf anywhere in the requested command
  Count = 6,
};

// Human-readable name for logging/`/status`. Total: unknown values (e.g.
// a corrupt enum from bad deserialization) map to "unknown" rather than
// indexing out of bounds.
const char* rejectReasonName(RejectReason reason);

struct EnvelopeResult {
  GaitCommand clamped;    // ONLY meaningful when accepted == true. On
                           // rejection this is a safe all-zero
                           // GaitCommand{} -- callers must check
                           // `accepted` first, not infer it from this.
  bool accepted = false;
  RejectReason reason = RejectReason::None;
  bool wasClamped = false;  // accepted, but one or more fields were
                              // modified to fit EnvelopeLimits -- lets
                              // the caller report achieved-vs-requested
                              // instead of silently lying about what the
                              // robot will actually do.
};

// Sane default cap on how often a Drive command is honored: comfortably
// above the firmware's own 50Hz control tick (dt == 0.02s, see
// firmware/host/test_gait.cpp), so it never fires on ordinary operation
// -- only on a runaway retry loop or a replayed burst.
constexpr float kDefaultMinDriveIntervalSec = 1.f / 200.f;  // 200 Hz

struct EnvelopeConfig {
  float minDriveIntervalSec = kDefaultMinDriveIntervalSec;
};

// Stateful safety gate: one instance per control link (i.e. per robot).
// Owns the E-stop latch, the last-accepted seq, and the last-accepted
// Drive timestamp. Pure C++11 -- takes every timestamp as an explicit
// parameter, calls no clock of its own (see core/README.md rule 4).
class Envelope {
 public:
  explicit Envelope(const EnvelopeConfig& config = EnvelopeConfig());

  // Evaluates one Drive-shaped request against, in order: the E-stop
  // latch, finiteness, seq freshness, the Drive rate cap, and the
  // (currently inert) obstacle seam -- then clamps to computeLimits()
  // and accepts. Every branch is total: there is no input for which
  // this does not return a well-formed EnvelopeResult.
  //
  // `nowSec` is a monotonic seconds timestamp from the CALLER's clock
  // (e.g. millis()/1000.f on the ESP32); core never reads a clock
  // itself. A non-finite `nowSec` is treated as a rate-limit failure
  // (fail safe) rather than trusted.
  //
  // seq/rate-limiter state is advanced only when this specific command
  // clears that specific check -- a rejected-as-Malformed command never
  // burns a seq slot, and a rejected-as-StaleSeq command never resets
  // the rate-limit clock -- so an attacker (or a bug) spamming garbage
  // cannot manufacture a rejection that opens a window for a later
  // replay.
  EnvelopeResult apply(const GaitCommand& requested, uint32_t seq,
                       float nowSec, ObstacleState obstacle,
                       const LegGeometry& geo, const GaitDef& def);

  // Latches immediately. From the next apply() call onward, EVERY
  // command is rejected with RejectReason::Estopped until clearEstop()
  // is called explicitly -- no Drive command, seq, or timestamp can
  // reach that check, let alone satisfy it, so the latch cannot be
  // accidentally bypassed by any command sequence.
  void triggerEstop();

  // The ONLY way to un-latch. Deliberately not reachable from apply()
  // or from any Command -- wiring a "clear" wire command to this is a
  // decision for the app layer (e.g. requiring an explicit operator
  // action), not something this module does implicitly. seq/rate-limit
  // state is intentionally NOT reset here: the first command accepted
  // after a clear still has to be strictly newer than whatever was last
  // accepted before the latch, so queued pre-latch traffic cannot replay
  // through immediately after a clear.
  void clearEstop();

  bool estopped() const { return estopLatched_; }

 private:
  EnvelopeConfig config_;
  bool estopLatched_ = false;
  bool haveSeq_ = false;
  uint32_t lastSeq_ = 0;
  bool haveDriveTime_ = false;
  float lastDriveTimeSec_ = 0.f;
};

}  // namespace core
}  // namespace sesame
