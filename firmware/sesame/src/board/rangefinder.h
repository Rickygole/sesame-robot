// rangefinder.h -- HC-SR04 ultrasonic distance, measured WITHOUT blocking.
//
// The obvious implementation, pulseIn(), busy-waits for the echo and can
// stall for up to ~30ms on a missed return. That is longer than the
// entire 20ms motion tick, so it would stall the servo update every time
// the sensor lost a reading -- which is precisely when the robot is most
// likely to be near something.
//
// Instead: fire the trigger, capture both echo edges in an ISR, and read
// the result on a later tick. The motion loop never waits.
//
// Readings are median-filtered over a short window. Ultrasonic sensors
// produce occasional wild outliers from off-axis reflections, and a
// single bad reading must not slam a walking robot to a halt.
//
// HARDWARE: the echo line MUST be level-shifted from 5V to 3.3V. See
// config.h. The ESP32 is not 5V tolerant.
#pragma once

#include <stdint.h>

namespace sesame {

constexpr uint16_t kRangeInvalidMm = 0xFFFF;  // no valid reading

class Rangefinder {
 public:
  bool begin();

  // Call once per motion tick. Fires a new ping when the previous one
  // has completed or timed out. Returns immediately, always.
  void tick(float dt);

  // Median-filtered distance, or kRangeInvalidMm if there is no recent
  // valid reading. Treat invalid as UNKNOWN, never as "clear" -- a
  // sensor that has stopped answering is not evidence of open space.
  uint16_t distanceMm() const;

  bool present() const { return present_; }

 private:
  static constexpr uint8_t kWindow = 5;

  bool present_ = false;
  float sinceLastPing_ = 0.f;
  uint16_t window_[kWindow] = {0};
  uint8_t windowLen_ = 0;
  uint8_t windowPos_ = 0;
  float sinceValid_ = 0.f;
};

}  // namespace sesame
