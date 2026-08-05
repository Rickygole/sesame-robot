// imu.h -- MPU6050 tilt sensing, used for ONE thing: levelling the body.
//
// Be clear about what an IMU can and cannot buy you on this mechanism.
//
// With 2 DOF per leg, a foot's reachable set is a torus, so you cannot
// hold the feet planted while re-orienting the body -- there is no
// foot-position-preserving balance to be had here, and no amount of
// sensing changes that. Dynamic balance recovery is likewise out: a
// crawl gait with three feet down is statically stable by construction,
// and the mechanism has no ankle to push with.
//
// What IS available, and is genuinely worth having: measure roll and
// pitch, then bias each leg's STANCE DEPTH so the body sits level on an
// uneven surface. On carpet, a sloped floor, or with one leg slightly
// mis-trimmed, that is the difference between a robot that looks drunk
// and one that looks deliberate. It is a small feature and it is the
// right one.
//
// The sensor lives on the same I2C bus as the OLED. Absent hardware is
// detected at begin() and the whole feature stays off -- the robot walks
// perfectly well without it.
#pragma once

#include <stdint.h>

namespace sesame {

struct Attitude {
  float rollDeg = 0.f;   // + = right side down
  float pitchDeg = 0.f;  // + = nose up
  bool valid = false;
};

class Imu {
 public:
  // Probes the bus. Returns false if nothing answers -- not an error,
  // just "no IMU fitted".
  bool begin();

  // Reads and updates the complementary filter. Call at the motion tick
  // rate. Cheap: one 14-byte I2C burst.
  void update(float dt);

  const Attitude& attitude() const { return attitude_; }
  bool present() const { return present_; }

  // Zeroes the current orientation. Call with the robot standing level
  // on a flat surface -- this cancels both mounting misalignment and
  // sensor bias in one step, which is far easier than trying to
  // characterise either separately.
  void calibrateLevel();

 private:
  bool present_ = false;
  Attitude attitude_;
  float rollOffset_ = 0.f;
  float pitchOffset_ = 0.f;
  float gyroBiasX_ = 0.f;
  float gyroBiasY_ = 0.f;
};

// Per-leg stance depth bias, in mm, that would level the body.
//
// Pure function of geometry and attitude, so it is host-testable. A leg
// on the low side of a tilt gets a SHALLOWER stance (foot closer to the
// body) and a leg on the high side gets a deeper one; the difference
// rotates the body back toward level.
//
// `gain` in [0,1] scales the correction. Well below 1 in practice --
// full correction fights the gait and oscillates.
void levelingBias(float rollDeg, float pitchDeg, float halfBodyLenMm,
                  float halfBodyWidMm, float gain, float outBiasMm[4]);

// Largest tilt considered recoverable. Past this the robot is falling or
// has been picked up, and the correct response is to stop rather than
// to flail a correction that cannot succeed.
constexpr float kMaxRecoverableTiltDeg = 35.f;

}  // namespace sesame
