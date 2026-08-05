#include "calibration.h"

#include <string.h>

namespace sesame {
namespace core {

namespace {
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

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
  return magic == kCalibrationMagic && crc == computeCrc();
}

int32_t degToUs(const JointCal& cal, float anatomicalDeg) {
  float d = anatomicalDeg + cal.trimDeg;
  if (cal.invert) {
    d = -d;
  }
  d = clampf(d, cal.minDeg, cal.maxDeg);
  float servoDeg = clampf(d + 90.f, 0.f, 180.f);
  const float t = servoDeg / 180.f;
  const float us = float(cal.usMin) + t * float(cal.usMax - cal.usMin);
  return int32_t(us + (us >= 0.f ? 0.5f : -0.5f));
}

float usToDeg(const JointCal& cal, int32_t us) {
  const float t = float(us - cal.usMin) / float(cal.usMax - cal.usMin);
  const float servoDeg = t * 180.f;
  float d = servoDeg - 90.f;
  if (cal.invert) {
    d = -d;
  }
  return d - cal.trimDeg;
}

}  // namespace core
}  // namespace sesame
