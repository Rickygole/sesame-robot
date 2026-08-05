// version_guard.h -- fail the build loudly on the wrong ESP32Servo.
//
// The firmware requires ESP32Servo 3.0.9 exactly. 3.1.0 reworked channel
// allocation and carries a bug where writing one servo can disturb others
// (madhephaestus/ESP32Servo#103). On a quadruped that presents as "the
// wrong leg moved", which is indistinguishable from a wiring fault and
// wastes hours.
//
// ESP32Servo exposes no usable version macro -- ESP32_Servo_VERSION is a
// stale legacy "1" unchanged since 2017, and the real version lives in
// library.properties where the preprocessor cannot see it. So we
// fingerprint a constant that actually changed at the boundary we care
// about: MAX_SERVOS is 16 throughout 3.0.x and 20 from 3.1.0 onward.
//
// Honest limitation: this cannot distinguish 3.0.9 from 3.0.8. It catches
// the boundary that matters; vendoring (see ESP32Servo/VENDORING.md)
// covers the rest.
#pragma once

#include "ESP32Servo/ESP32Servo.h"

#ifndef MAX_SERVOS
#error "ESP32Servo/ESP32Servo.h did not define MAX_SERVOS. Wrong or missing library."
#endif

static_assert(MAX_SERVOS == 16,
              "Wrong ESP32Servo version. This firmware requires the vendored "
              "3.0.9 at src/vendor/ESP32Servo/. MAX_SERVOS==20 means 3.1.0+, "
              "which has issue #103 (writing one servo disturbs others). "
              "Do NOT install ESP32Servo via Library Manager.");
