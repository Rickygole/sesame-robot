#include "rangefinder.h"

#include <Arduino.h>

#include "../../config.h"

namespace sesame {

namespace {

// Written by the ISR, read by the motion loop. volatile is necessary but
// not sufficient on its own -- echoDone_ is written LAST and read FIRST,
// which is what makes the handoff safe without a lock.
volatile uint32_t g_echoStartUs = 0;
volatile uint32_t g_echoEndUs = 0;
volatile bool g_echoDone = false;
volatile bool g_echoPending = false;

// Ping interval. The HC-SR04 needs ~60ms between pings or the previous
// burst's echoes contaminate the next reading.
constexpr float kPingIntervalSec = 0.07f;

// Beyond ~4m the sensor is unreliable; 25ms of flight is about 4.3m.
constexpr uint32_t kEchoTimeoutUs = 25000;

// Speed of sound, there and back: 343 m/s => 0.1715 mm per microsecond.
constexpr float kMmPerUs = 0.1715f;

void IRAM_ATTR echoIsr() {
  const uint32_t now = micros();
  if (digitalRead(kRangeEchoPin) == HIGH) {
    g_echoStartUs = now;
    g_echoDone = false;
  } else if (g_echoStartUs != 0) {
    g_echoEndUs = now;
    g_echoDone = true;  // written last: the loop polls this to publish
  }
}

uint16_t medianOf(const uint16_t* values, uint8_t n) {
  // Insertion sort into a small scratch copy. n <= 5, so this is
  // cheaper than anything cleverer and has no allocation.
  uint16_t s[8];
  for (uint8_t i = 0; i < n; ++i) {
    uint16_t v = values[i];
    int8_t j = int8_t(i) - 1;
    while (j >= 0 && s[j] > v) {
      s[j + 1] = s[j];
      --j;
    }
    s[j + 1] = v;
  }
  return s[n / 2];
}

}  // namespace

bool Rangefinder::begin() {
  pinMode(kRangeTrigPin, OUTPUT);
  digitalWrite(kRangeTrigPin, LOW);
  pinMode(kRangeEchoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kRangeEchoPin), echoIsr, CHANGE);

  // There is no way to probe an HC-SR04 for presence -- it is a dumb
  // two-wire device. Assume fitted; distanceMm() reports invalid if
  // nothing ever answers, and the caller treats invalid as UNKNOWN
  // rather than clear.
  present_ = true;
  windowLen_ = 0;
  windowPos_ = 0;
  sinceValid_ = 0.f;
  return true;
}

void Rangefinder::tick(float dt) {
  if (!present_) {
    return;
  }
  sinceLastPing_ += dt;
  sinceValid_ += dt;

  // Collect a completed echo from the ISR.
  if (g_echoDone) {
    const uint32_t start = g_echoStartUs;
    const uint32_t end = g_echoEndUs;
    g_echoDone = false;
    g_echoPending = false;
    const uint32_t widthUs = end - start;  // wraps correctly on overflow
    if (widthUs > 0 && widthUs < kEchoTimeoutUs) {
      const uint16_t mm = uint16_t(float(widthUs) * kMmPerUs);
      window_[windowPos_] = mm;
      windowPos_ = uint8_t((windowPos_ + 1) % kWindow);
      if (windowLen_ < kWindow) {
        ++windowLen_;
      }
      sinceValid_ = 0.f;
    }
  }

  if (sinceLastPing_ >= kPingIntervalSec) {
    sinceLastPing_ = 0.f;
    g_echoPending = true;
    g_echoStartUs = 0;
    // 10us trigger pulse. Blocking for 10 microseconds is fine; blocking
    // for the echo would not be.
    digitalWrite(kRangeTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(kRangeTrigPin, LOW);
  }
}

uint16_t Rangefinder::distanceMm() const {
  // Stale readings must not masquerade as current ones. If the sensor
  // has gone quiet for a third of a second, say UNKNOWN.
  if (windowLen_ == 0 || sinceValid_ > 0.35f) {
    return kRangeInvalidMm;
  }
  return medianOf(window_, windowLen_);
}

}  // namespace sesame
