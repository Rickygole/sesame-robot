// mathutil.h -- tiny NaN-safe scalar helpers shared across core/.
//
// This is the single definition of clampf() for the whole motion core.
// Previously slew.cpp and calibration.cpp each carried their own
// private copy of `v<lo ? lo : (v>hi ? hi : v)`, and BOTH copies let a
// NaN input pass straight through unchanged (NaN compares false
// against everything, so neither branch of the clamp ever fires). A
// NaN that reaches a servo command is undefined behavior once it hits
// an int32_t cast (see calibration.cpp::degToUs), so this is treated
// as a safety bug, not a style nit.
#pragma once

#include <cmath>
#include <stdint.h>

namespace sesame {
namespace core {

// Absolute physical pulse-width envelope for the servos, INDEPENDENT of
// any (possibly corrupt or attacker-supplied) calibration data. No code
// path may ever emit a pulse width outside this range, for any input.
//
// The project's nominal MG90S mapping is 732..2929us; this envelope is
// deliberately a little wider so it bounds the legitimate range without
// clipping it, while still excluding values that would drive a servo
// into a hard end stop.
constexpr int32_t kAbsServoUsMin = 500;
constexpr int32_t kAbsServoUsMax = 3000;

inline bool isFiniteF(float v) {
  return !std::isnan(v) && !std::isinf(v);
}

// Clamp v into [lo, hi]. The NaN fallback is an EXPLICIT parameter with
// no default, on purpose: "what is safe when this value is garbage?" has
// a different answer at every call site. For a servo angle, silently
// returning `lo` would slam the joint to an end stop -- neutral is the
// safe answer there, while for a scale factor zero is. Forcing the
// caller to name it keeps that decision visible in the code.
//
// +Inf/-Inf need no special case: they compare correctly against finite
// bounds and clamp to hi/lo as you would want.
inline float clampf(float v, float lo, float hi, float nanFallback) {
  if (std::isnan(v)) {
    return nanFallback;
  }
  return v < lo ? lo : (v > hi ? hi : v);
}

inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace core
}  // namespace sesame
