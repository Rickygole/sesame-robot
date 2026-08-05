#include "calibration.h"

#include <string.h>

#include "mathutil.h"

namespace sesame {
namespace core {

namespace {
// Appends raw bytes of a little-endian-encoded value into buf, returns
// the new offset. Kept explicit (rather than reinterpret_cast'ing whole
// structs) so the CRC is independent of compiler struct padding and
// portable between this host build and the ESP32 target.
uint32_t appendU32(uint8_t* buf, uint32_t off, uint32_t v) {
  buf[off + 0] = uint8_t(v & 0xFF);
  buf[off + 1] = uint8_t((v >> 8) & 0xFF);
  buf[off + 2] = uint8_t((v >> 16) & 0xFF);
  buf[off + 3] = uint8_t((v >> 24) & 0xFF);
  return off + 4;
}
uint32_t appendI16(uint8_t* buf, uint32_t off, int16_t v) {
  uint16_t u = uint16_t(v);
  buf[off + 0] = uint8_t(u & 0xFF);
  buf[off + 1] = uint8_t((u >> 8) & 0xFF);
  return off + 2;
}
uint32_t appendF32(uint8_t* buf, uint32_t off, float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  return appendU32(buf, off, bits);
}
uint32_t appendU8(uint8_t* buf, uint32_t off, uint8_t v) {
  buf[off] = v;
  return off + 1;
}
}  // namespace

uint32_t crc32(const uint8_t* data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = uint32_t(-int32_t(crc & 1u));
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

uint32_t Calibration::computeCrc() const {
  // magic(4) + version(4) + kJointCount * (usMin(2)+usMax(2)+trim(4)+invert(1)+minDeg(4)+maxDeg(4))
  constexpr uint32_t kPerJoint = 2 + 2 + 4 + 1 + 4 + 4;
  constexpr uint32_t kBufSize = 4 + 4 + kJointCount * kPerJoint;
  uint8_t buf[kBufSize];
  uint32_t off = 0;
  off = appendU32(buf, off, magic);
  off = appendU32(buf, off, version);
  for (uint8_t i = 0; i < kJointCount; ++i) {
    const JointCal& j = joint[i];
    off = appendI16(buf, off, j.usMin);
    off = appendI16(buf, off, j.usMax);
    off = appendF32(buf, off, j.trimDeg);
    off = appendU8(buf, off, j.invert ? 1 : 0);
    off = appendF32(buf, off, j.minDeg);
    off = appendF32(buf, off, j.maxDeg);
  }
  return crc32(buf, off);
}

bool Calibration::isValid() const {
  // The version MUST be compared against the firmware's expected
  // constant, not merely round-tripped through the CRC. computeCrc()
  // serializes whatever version is *stored*, so a blob written by a
  // future firmware with a different field meaning but the same byte
  // layout is internally self-consistent and would otherwise validate.
  return magic == kCalibrationMagic && version == kCalibrationVersion &&
         crc == computeCrc();
}

int32_t degToUs(const JointCal& cal, float anatomicalDeg) {
  // Nothing below this line may trust `cal` or `anatomicalDeg`. Both can
  // arrive corrupt (bad flash, a malformed SetCal command, a NaN that
  // leaked through an upstream guard), and the return value drives a
  // physical servo. Fail to NEUTRAL, never to an end stop.
  const int32_t lo = clampI32(int32_t(cal.usMin), kAbsServoUsMin, kAbsServoUsMax);
  const int32_t hi = clampI32(int32_t(cal.usMax), kAbsServoUsMin, kAbsServoUsMax);
  const int32_t absCenter = (kAbsServoUsMin + kAbsServoUsMax) / 2;
  if (lo >= hi) {
    return absCenter;  // degenerate/inverted calibration span
  }
  const int32_t center = (lo + hi) / 2;

  if (!isFiniteF(anatomicalDeg) || !isFiniteF(cal.trimDeg)) {
    return center;
  }
  float d = anatomicalDeg + cal.trimDeg;
  if (cal.invert) {
    d = -d;
  }

  // The soft limits themselves may be non-finite or inverted.
  float mn = isFiniteF(cal.minDeg) ? cal.minDeg : -90.f;
  float mx = isFiniteF(cal.maxDeg) ? cal.maxDeg : 90.f;
  if (mn > mx) {
    mn = -90.f;
    mx = 90.f;
  }

  d = clampf(d, mn, mx, 0.f);
  const float servoDeg = clampf(d + 90.f, 0.f, 180.f, 90.f);
  const float us = float(lo) + (servoDeg / 180.f) * float(hi - lo);

  // us is provably finite and within [lo,hi] by construction, so the
  // cast below cannot be UB -- but clamp the result anyway. This is the
  // last line of defense before a pulse width reaches a servo.
  return clampI32(int32_t(us + 0.5f), lo, hi);
}

float usToDeg(const JointCal& cal, int32_t us) {
  const int32_t span = int32_t(cal.usMax) - int32_t(cal.usMin);
  if (span == 0) {
    return 0.f;  // degenerate span: no meaningful inverse
  }
  const float t = float(us - cal.usMin) / float(span);
  const float servoDeg = t * 180.f;
  float d = servoDeg - 90.f;
  if (cal.invert) {
    d = -d;
  }
  return d - cal.trimDeg;
}

}  // namespace core
}  // namespace sesame
