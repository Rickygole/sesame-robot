// servo_bank.h -- the only place in the firmware that writes a PWM duty.
//
// Deliberately POSE-oriented, not joint-oriented. There is no public
// single-joint write in the motion path, because the power budget is only
// enforceable if every servo command flows through one function. A stray
// setServoAngle(i, deg) somewhere would let a caller command a large step
// on one joint and brown out the 5V rail -- the exact failure this design
// exists to prevent.
//
// Single-joint control DOES exist, but only via beginCalibrationMode(),
// which takes exclusive ownership and is not usable while a gait runs.
#pragma once

#include <stdint.h>

#include "../core/calibration.h"
#include "../core/pose.h"

namespace sesame {

class ServoBank {
 public:
  // Allocates LEDC timers and attaches all 8 channels.
  //
  // Only 2 hardware timers are requested, not 4. ESP32Servo maps LEDC
  // channel j to timer (j/2)%4 and allows 4 channels per timer, so 8
  // servos at a single frequency need exactly 2. Reserving all four
  // needlessly starves tone()/analogWrite().
  bool begin(const core::Calibration& cal);

  void setCalibration(const core::Calibration& cal);
  const core::Calibration& calibration() const { return cal_; }

  // Convert an anatomical pose to pulse widths and write all 8 channels.
  // Calibration clamps every value into the physical envelope, so this
  // cannot emit an unsafe pulse width regardless of what it is handed.
  void writePose(const core::JointPose& pose);

  const core::JointPose& lastWritten() const { return last_; }

  // Drive to a pose ONE JOINT AT A TIME with a gap between each.
  //
  // This is the only blocking API in the class and it belongs in setup()
  // only. At power-on the servos are at unknown physical positions, so
  // the first command is necessarily a large step for all eight at once
  // -- precisely the simultaneous-inrush case the slew limiter exists to
  // avoid, and it cannot help here because there is no known starting
  // pose to interpolate from. Sequencing is the only defense.
  void homeSequenced(const core::JointPose& pose, uint16_t stepMs);

  void detachAll();
  void attachAll();
  bool attached() const { return attached_; }

  // Boot self-test for ESP32Servo issue #103 (writing one servo disturbs
  // others). Writes a distinct pulse width to each channel, reads all
  // eight back, and reports the first channel whose readback does not
  // match what it was given.
  //
  // This tests the actual symptom rather than trusting a version macro,
  // so it survives any future library change. Returns true if clean;
  // on failure *badChannel receives the offending WIRE channel.
  bool selfTest(uint8_t* badChannel);

  // Diagnostic form: writes the same distinct pulse widths and reports
  // what EVERY channel read back, rather than stopping at the first
  // mismatch. "Channel 3 is wrong" and "channels 3-7 are all wrong" have
  // completely different causes, and the short form cannot tell them
  // apart.
  void selfTestReport(int32_t outWant[core::kJointCount],
                      int32_t outGot[core::kJointCount]);

 private:
  core::Calibration cal_;
  core::JointPose last_;
  bool attached_ = false;
};

}  // namespace sesame
