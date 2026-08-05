// calibration.h -- per-joint anatomical-degrees <-> servo-microseconds
// mapping, plus a CRC32'd struct suitable for storing in NVS.
#pragma once

#include <stdint.h>

#include "types.h"

namespace sesame {
namespace core {

struct JointCal {
  int16_t usMin = 732;    // MG90S pulse width at servo-space 0 deg
  int16_t usMax = 2929;   // MG90S pulse width at servo-space 180 deg
  float trimDeg = 0.f;    // added to anatomical deg before mapping
  bool invert = false;    // mirrors anatomical deg about 0 (left/right mounting)
  float minDeg = -60.f;   // soft limit, anatomical degrees
  float maxDeg = 90.f;    // soft limit, anatomical degrees
};

constexpr uint32_t kCalibrationMagic = 0x53534D31u;  // "SSM1"
constexpr uint32_t kCalibrationVersion = 1u;

struct Calibration {
  JointCal joint[kJointCount];
  uint32_t magic = kCalibrationMagic;
  uint32_t version = kCalibrationVersion;
  uint32_t crc = 0;

  // CRC32 over magic + version + every joint field, in a fixed byte
  // order that does NOT depend on struct padding/layout (so it is
  // portable between this host build and the ESP32 target). Does NOT
  // include the crc field itself.
  uint32_t computeCrc() const;

  // magic matches AND crc matches computeCrc().
  bool isValid() const;
};

// Anatomical degrees -> servo pulse width in microseconds:
//   1) deg += trimDeg
//   2) if invert: deg = -deg          (mirrors about the anatomical 0)
//   3) deg = clamp(deg, minDeg, maxDeg)
//   4) servoDeg = deg + 90            (anatomical 0 == servo-space 90, the
//                                       servo's mechanical center)
//   5) map servoDeg linearly from [0,180] onto [usMin,usMax]
int32_t degToUs(const JointCal& cal, float anatomicalDeg);

// Inverse of the above, EXCLUDING the soft-limit clamp (step 3), which
// is inherently not invertible. Exists to validate the mapping is
// monotone and that `invert` is an involution.
float usToDeg(const JointCal& cal, int32_t us);

uint32_t crc32(const uint8_t* data, uint32_t len);

}  // namespace core
}  // namespace sesame
