#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "../../config.h"
#include "../core/types.h"

namespace sesame {

namespace {

constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegAccelXout = 0x3B;

// MPU6050 defaults: accel +/-2g (16384 LSB/g), gyro +/-250 deg/s (131 LSB/dps)
constexpr float kAccelScale = 1.f / 16384.f;
constexpr float kGyroScale = 1.f / 131.f;

// Complementary filter weight. Gyro is trusted short-term (no drift over
// a tick) and accelerometer long-term (no drift over minutes). 0.98 puts
// the crossover around a second, which suits a walking robot: leg impacts
// spike the accelerometer, and this rejects them without letting gyro
// drift accumulate.
constexpr float kAlpha = 0.98f;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kImuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(kImuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const uint8_t got = Wire.requestFrom(uint8_t(kImuAddr), len);
  if (got != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    buf[i] = uint8_t(Wire.read());
  }
  return true;
}

int16_t be16(const uint8_t* p) {
  return int16_t((uint16_t(p[0]) << 8) | p[1]);
}

}  // namespace

bool Imu::begin() {
  // Wire.begin() is owned by Display -- the IMU shares that bus and must
  // not re-initialise it.
  uint8_t who = 0;
  if (!readRegs(kRegWhoAmI, &who, 1)) {
    present_ = false;
    return false;
  }
  // 0x68 is the MPU6050; MPU6500/9250 answer 0x70/0x71 and are close
  // enough at this level of use.
  if (who != 0x68 && who != 0x70 && who != 0x71) {
    present_ = false;
    return false;
  }
  if (!writeReg(kRegPwrMgmt1, 0x00)) {  // wake from sleep
    present_ = false;
    return false;
  }
  delay(50);
  present_ = true;
  attitude_.valid = false;
  return true;
}

void Imu::update(float dt) {
  if (!present_ || dt <= 0.f || dt > 0.5f) {
    return;
  }
  uint8_t raw[14];
  if (!readRegs(kRegAccelXout, raw, 14)) {
    // A dropped read is not fatal -- keep the last attitude rather than
    // zeroing, which would jerk the levelling correction.
    return;
  }

  const float ax = float(be16(raw + 0)) * kAccelScale;
  const float ay = float(be16(raw + 2)) * kAccelScale;
  const float az = float(be16(raw + 4)) * kAccelScale;
  const float gx = float(be16(raw + 8)) * kGyroScale - gyroBiasX_;
  const float gy = float(be16(raw + 10)) * kGyroScale - gyroBiasY_;

  // Accelerometer gives absolute tilt but is noisy under leg impacts.
  const float accRoll = atan2f(ay, sqrtf(az * az + ax * ax)) * 57.29578f;
  const float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

  if (!attitude_.valid) {
    attitude_.rollDeg = accRoll;
    attitude_.pitchDeg = accPitch;
    attitude_.valid = true;
    return;
  }

  // Gyro integration for the short term, accelerometer to stop it
  // drifting. Standard complementary filter -- a Kalman filter would buy
  // very little here and cost a lot of tuning.
  attitude_.rollDeg =
      kAlpha * (attitude_.rollDeg + gx * dt) + (1.f - kAlpha) * accRoll;
  attitude_.pitchDeg =
      kAlpha * (attitude_.pitchDeg + gy * dt) + (1.f - kAlpha) * accPitch;
}

void Imu::calibrateLevel() {
  if (!present_) {
    return;
  }
  // Average a short burst: cancels mounting misalignment and gyro bias
  // together, which is much easier than characterising either alone.
  float rollSum = 0.f, pitchSum = 0.f, gxSum = 0.f, gySum = 0.f;
  const int kSamples = 50;
  int taken = 0;
  for (int i = 0; i < kSamples; ++i) {
    uint8_t raw[14];
    if (!readRegs(kRegAccelXout, raw, 14)) {
      continue;
    }
    const float ax = float(be16(raw + 0)) * kAccelScale;
    const float ay = float(be16(raw + 2)) * kAccelScale;
    const float az = float(be16(raw + 4)) * kAccelScale;
    rollSum += atan2f(ay, sqrtf(az * az + ax * ax)) * 57.29578f;
    pitchSum += atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    gxSum += float(be16(raw + 8)) * kGyroScale;
    gySum += float(be16(raw + 10)) * kGyroScale;
    ++taken;
    delay(4);
  }
  if (taken == 0) {
    return;
  }
  rollOffset_ = rollSum / float(taken);
  pitchOffset_ = pitchSum / float(taken);
  gyroBiasX_ = gxSum / float(taken);
  gyroBiasY_ = gySum / float(taken);
  attitude_.valid = false;  // re-seed on the next update
}

void levelingBias(float rollDeg, float pitchDeg, float halfBodyLenMm,
                  float halfBodyWidMm, float gain, float outBiasMm[4]) {
  // Small-angle: the height error at a hip is its lever arm times the
  // tilt in radians. Exact enough well past any tilt worth correcting.
  const float kDegToRad = 0.0174532925f;
  const float rollRad = rollDeg * kDegToRad;
  const float pitchRad = pitchDeg * kDegToRad;

  for (uint8_t i = 0; i < core::kLegCount; ++i) {
    const core::Leg leg = core::Leg(i);
    const bool front = (leg == core::Leg::FR || leg == core::Leg::FL);
    const bool right = (leg == core::Leg::FR || leg == core::Leg::RR);
    const float x = front ? halfBodyLenMm : -halfBodyLenMm;
    const float y = right ? halfBodyWidMm : -halfBodyWidMm;

    // How far this hip currently sits above the level plane. Extending
    // that leg by the same amount pushes the corner back down.
    const float heightErr = (y * rollRad) - (x * pitchRad);
    outBiasMm[i] = gain * heightErr;
  }
}

}  // namespace sesame
