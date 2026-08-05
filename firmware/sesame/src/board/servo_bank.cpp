#include "servo_bank.h"

#include <Arduino.h>

#include "../../config.h"
#include "../core/types.h"
#include "../vendor/version_guard.h"

namespace sesame {

namespace {

::Servo g_servos[core::kJointCount];

// Wire channel -> logical joint index. This is the ONE translation from
// wire order to logical order in the entire firmware; everything above
// this layer speaks logical order (leg*2 + joint) and never has to know
// that channels 4 and 5 look swapped.
inline uint8_t logicalForChannel(uint8_t ch) {
  const core::JointAddr addr = core::kChannelMap[ch];
  return core::logicalIndex(addr.leg, addr.joint);
}

}  // namespace

bool ServoBank::begin(const core::Calibration& cal) {
  cal_ = cal;

  // 8 servos at one frequency occupy exactly 2 LEDC timers under
  // ESP32Servo's allocation scheme. Leave timers 2 and 3 free.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const uint8_t logical = logicalForChannel(ch);
    g_servos[ch].setPeriodHertz(kPwmHz);
    g_servos[ch].attach(kServoPins[ch], cal_.joint[logical].usMin,
                        cal_.joint[logical].usMax);
  }
  attached_ = true;

  for (uint8_t i = 0; i < core::kJointCount; ++i) {
    last_.deg[i] = 0.f;
  }
  return true;
}

void ServoBank::setCalibration(const core::Calibration& cal) { cal_ = cal; }

void ServoBank::writePose(const core::JointPose& pose) {
  if (!attached_) {
    return;
  }
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const uint8_t logical = logicalForChannel(ch);
    // degToUs is total: it bounds the result to the physical envelope for
    // ANY input, including NaN and a corrupt calibration. Nothing further
    // is needed here to keep the pulse width safe.
    const int32_t us = core::degToUs(cal_.joint[logical], pose.deg[logical]);
    g_servos[ch].writeMicroseconds(uint32_t(us));
    last_.deg[logical] = pose.deg[logical];
  }
}

void ServoBank::homeSequenced(const core::JointPose& pose, uint16_t stepMs) {
  if (!attached_) {
    return;
  }
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const uint8_t logical = logicalForChannel(ch);
    const int32_t us = core::degToUs(cal_.joint[logical], pose.deg[logical]);
    g_servos[ch].writeMicroseconds(uint32_t(us));
    last_.deg[logical] = pose.deg[logical];
    delay(stepMs);  // blocking on purpose; setup() only
  }
}

void ServoBank::detachAll() {
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    g_servos[ch].detach();
  }
  attached_ = false;
}

void ServoBank::attachAll() {
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const uint8_t logical = logicalForChannel(ch);
    g_servos[ch].setPeriodHertz(kPwmHz);
    g_servos[ch].attach(kServoPins[ch], cal_.joint[logical].usMin,
                        cal_.joint[logical].usMax);
  }
  attached_ = true;
}

bool ServoBank::selfTest(uint8_t* badChannel) {
  if (!attached_) {
    return false;
  }
  // Distinct, well-separated pulse widths so a cross-channel disturbance
  // cannot be mistaken for rounding. Spread across the usable span.
  int32_t want[core::kJointCount];
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    want[ch] = 1200 + int32_t(ch) * 60;  // 1200..1620us
    g_servos[ch].writeMicroseconds(uint32_t(want[ch]));
  }
  // Read back only AFTER all writes -- the failure mode being tested is
  // that a later write disturbs an earlier channel.
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const int32_t got = int32_t(g_servos[ch].readMicroseconds());
    if (got < want[ch] - 12 || got > want[ch] + 12) {
      if (badChannel != nullptr) {
        *badChannel = ch;
      }
      return false;
    }
  }
  return true;
}

}  // namespace sesame
