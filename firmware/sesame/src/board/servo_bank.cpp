#include "servo_bank.h"

#include <Arduino.h>

#include "../../config.h"
#include "../core/types.h"
#include "../vendor/version_guard.h"

namespace sesame {

namespace {

::Servo g_servos[core::kJointCount];

// --- Boot self-test parameters ---------------------------------------
//
// The test writes a distinct pulse width to every channel and reads them
// all back, to catch the ESP32Servo failure where writing one servo
// disturbs ANOTHER (madhephaestus/ESP32Servo#103).
//
// TOLERANCE: measured on real hardware, writeMicroseconds -> LEDC duty
// counts -> readMicroseconds loses about 1%, growing with the value:
// 1200 reads back 1191, 1620 reads back 1601. That is ordinary
// quantization, not a fault. An earlier fixed +/-12us tolerance was
// therefore tighter than the round trip's own accuracy and reported a
// FATAL on healthy hardware -- which is worse than no test, because it
// sends you hunting a wiring fault that does not exist.
//
// 40us is comfortably above the ~20us worst-case quantization error and
// far below the failure being detected: genuine cross-talk makes a
// channel read back a DIFFERENT channel's value, which the 150us spacing
// below makes a >=150us discrepancy.
//
// Also note this is ~2 degrees of servo travel, well inside an MG90S's
// own 1-2 degree deadband -- an error this small is not physically
// observable anyway.
constexpr int32_t kSelfTestBaseUs = 1000;
constexpr int32_t kSelfTestStepUs = 150;   // 1000..2050us
constexpr int32_t kSelfTestToleranceUs = 40;

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

void ServoBank::selfTestReport(int32_t outWant[core::kJointCount],
                               int32_t outGot[core::kJointCount]) {
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    outWant[ch] = kSelfTestBaseUs + int32_t(ch) * kSelfTestStepUs;
    g_servos[ch].writeMicroseconds(uint32_t(outWant[ch]));
  }
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    outGot[ch] = int32_t(g_servos[ch].readMicroseconds());
  }
}

bool ServoBank::selfTest(uint8_t* badChannel) {
  if (!attached_) {
    return false;
  }
  int32_t want[core::kJointCount];
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    want[ch] = kSelfTestBaseUs + int32_t(ch) * kSelfTestStepUs;
    g_servos[ch].writeMicroseconds(uint32_t(want[ch]));
  }
  // Read back only AFTER all writes -- the failure mode being tested is
  // that a later write disturbs an EARLIER channel.
  for (uint8_t ch = 0; ch < core::kJointCount; ++ch) {
    const int32_t got = int32_t(g_servos[ch].readMicroseconds());
    if (got < want[ch] - kSelfTestToleranceUs ||
        got > want[ch] + kSelfTestToleranceUs) {
      if (badChannel != nullptr) {
        *badChannel = ch;
      }
      return false;
    }
  }
  return true;
}

}  // namespace sesame
