#include "envelope.h"

#include <cmath>

#include "mathutil.h"

namespace sesame {
namespace core {

namespace {
constexpr float kPi = 3.14159265358979323846f;
inline float degToRad(float deg) { return deg * (kPi / 180.f); }
inline float radToDeg(float rad) { return rad * (180.f / kPi); }

bool commandIsFinite(const GaitCommand& cmd) {
  return isFiniteF(cmd.vxMmPerSec) && isFiniteF(cmd.omegaDegPerSec) &&
         isFiniteF(cmd.bodyHeightMm) && isFiniteF(cmd.stepHeightMm) &&
         isFiniteF(cmd.maxStrideMm);
}

// Standard sequence-number-newer-than test (same idea as TCP sequence
// comparison): works across a uint32_t wraparound as long as successive
// seqs are never more than 2^31 apart, which holds for any sane sender.
bool seqIsNewer(uint32_t seq, uint32_t lastSeq) {
  return int32_t(seq - lastSeq) > 0;
}
}  // namespace

EnvelopeLimits computeLimits(const LegGeometry& geo, const GaitDef& def,
                              float bodyHeightMm) {
  EnvelopeLimits lim;

  const float loH = 0.15f * geo.legMm;
  const float hiH = 0.95f * geo.legMm;
  lim.bodyHeightMinMm = loH;
  lim.bodyHeightMaxMm = hiH;

  // Achieved height, exactly as gait.cpp clamps it, so everything
  // downstream is derived at the height the gait will actually use.
  const float h = clampf(bodyHeightMm, loH, hiH, 0.5f * (loH + hiH));
  lim.stepHeightMaxMm = h;

  const float phiSt = asinf(clampf(h / geo.legMm, -1.f, 1.f, 0.f));
  const float rSt = geo.coxaMm + geo.legMm * cosf(phiSt);
  lim.stanceRadiusMm = rSt;

  const float halfSweepRad = degToRad(kMaxHalfSweepDeg);
  const float duty = def.dutyFactor;
  const float freqHz = def.freqHz;

  if (duty > 1e-6f) {
    lim.vxMaxMmPerSec = halfSweepRad * 2.f * freqHz * rSt / duty;
    lim.omegaMaxDegPerSec = radToDeg(halfSweepRad * 2.f * freqHz / duty);
  } else {
    // Degenerate gait def (duty == 0, no stance phase at all): nothing
    // is achievable without violating the sweep cap.
    lim.vxMaxMmPerSec = 0.f;
    lim.omegaMaxDegPerSec = 0.f;
  }

  return lim;
}

const char* rejectReasonName(RejectReason reason) {
  switch (reason) {
    case RejectReason::None:
      return "none";
    case RejectReason::Estopped:
      return "estopped";
    case RejectReason::Obstacle:
      return "obstacle";
    case RejectReason::RateLimited:
      return "rate_limited";
    case RejectReason::StaleSeq:
      return "stale_seq";
    case RejectReason::Malformed:
      return "malformed";
    case RejectReason::Count:
      break;
  }
  return "unknown";
}

Envelope::Envelope(const EnvelopeConfig& config) : config_(config) {}

void Envelope::triggerEstop() { estopLatched_ = true; }

void Envelope::clearEstop() { estopLatched_ = false; }

EnvelopeResult Envelope::apply(const GaitCommand& requested, uint32_t seq,
                               float nowSec, ObstacleState obstacle,
                               const LegGeometry& geo, const GaitDef& def) {
  EnvelopeResult result;
  result.clamped = GaitCommand();  // safe all-zero default unless accepted

  // ---- 1. The latch. Nothing -- not a fresh seq, not a well-formed
  // command, not obstacle-clear -- can reach past this. ----
  if (estopLatched_) {
    result.reason = RejectReason::Estopped;
    return result;
  }

  // ---- 2. Non-finite rejection. Never clamped into something
  // plausible -- and never allowed to burn a seq slot or the rate-limit
  // clock, so a garbage packet cannot open a replay window either. ----
  if (!commandIsFinite(requested)) {
    result.reason = RejectReason::Malformed;
    return result;
  }

  // ---- 3. Monotonic seq: reject stale/replayed/reordered commands. ----
  if (haveSeq_ && !seqIsNewer(seq, lastSeq_)) {
    result.reason = RejectReason::StaleSeq;
    return result;
  }
  lastSeq_ = seq;
  haveSeq_ = true;

  // ---- 4. Rate limit. A non-finite timestamp is treated as a failure
  // (fail safe) rather than trusted -- NaN compares false against
  // everything, so an ungated check here would let it slip through. ----
  if (!isFiniteF(nowSec) ||
      (haveDriveTime_ && (nowSec - lastDriveTimeSec_) < config_.minDriveIntervalSec)) {
    result.reason = RejectReason::RateLimited;
    return result;
  }
  lastDriveTimeSec_ = nowSec;
  haveDriveTime_ = true;

  // ---- 5. Obstacle seam. SHIPPED DARK -- see envelope.h. Currently
  // always Clear, so this branch is unreachable in practice; it exists
  // so the call shape is correct once a sensor exists. ----
  if (obstacle != ObstacleState::Clear) {
    result.reason = RejectReason::Obstacle;
    return result;
  }

  // ---- 6. Clamp to what the machine can physically do. ----
  const EnvelopeLimits limits = computeLimits(geo, def, requested.bodyHeightMm);

  GaitCommand clamped;
  clamped.bodyHeightMm = clampf(requested.bodyHeightMm, limits.bodyHeightMinMm,
                                 limits.bodyHeightMaxMm,
                                 0.5f * (limits.bodyHeightMinMm + limits.bodyHeightMaxMm));
  clamped.stepHeightMm =
      clampf(requested.stepHeightMm, 0.f, limits.stepHeightMaxMm, 0.f);
  clamped.vxMmPerSec = clampf(requested.vxMmPerSec, -limits.vxMaxMmPerSec,
                               limits.vxMaxMmPerSec, 0.f);
  clamped.omegaDegPerSec =
      clampf(requested.omegaDegPerSec, -limits.omegaMaxDegPerSec,
             limits.omegaMaxDegPerSec, 0.f);
  // maxStrideMm has no envelope-level cap of its own (already validated
  // finite above); gait.cpp folds it into the same per-tick sweep clamp
  // as kMaxHalfSweepDeg, so it is passed through unmodified.
  clamped.maxStrideMm = requested.maxStrideMm;

  const bool wasClamped = (clamped.bodyHeightMm != requested.bodyHeightMm) ||
                           (clamped.stepHeightMm != requested.stepHeightMm) ||
                           (clamped.vxMmPerSec != requested.vxMmPerSec) ||
                           (clamped.omegaDegPerSec != requested.omegaDegPerSec);

  result.clamped = clamped;
  result.accepted = true;
  result.reason = RejectReason::None;
  result.wasClamped = wasClamped;
  return result;
}

}  // namespace core
}  // namespace sesame
