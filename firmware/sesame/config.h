// config.h -- board pinout and timing constants. No logic lives here.
//
// This is the ONE file you edit when moving to different hardware.
#pragma once

#include <stdint.h>

namespace sesame {

// --- Servo channels -------------------------------------------------
//
// Indexed by WIRE channel, matching kChannelMap in src/core/types.h.
// The apparent oddity at indices 4 and 5 is real and intentional: the
// hip block is ordered [FR, RR, FL, RL] but the knee block is ordered
// [RR, FR, FL, RL]. See src/core/types.h for the derivation.
//
//   ch  GPIO  upstream name  leg          joint
//    0    15  R1             front-right  hip
//    1     2  R2             rear-right   hip
//    2    23  L1             front-left   hip
//    3    19  L2             rear-left    hip
//    4     4  R4             rear-right   knee
//    5    16  R3             front-right  knee
//    6    17  L3             front-left   knee
//    7    18  L4             rear-left    knee
//
// WARNING: GPIO 2 and GPIO 15 are ESP32 boot strapping pins. If an
// upload fails or the board boot-loops, unplug the servos on channels
// 0 and 1 and retry BEFORE suspecting cables or drivers. Firmware
// cannot fix this -- straps are latched before any code runs.
constexpr uint8_t kServoPins[8] = {15, 2, 23, 19, 4, 16, 17, 18};

// --- I2C (OLED, and later the IMU) ----------------------------------
constexpr uint8_t kI2cSda = 21;
constexpr uint8_t kI2cScl = 22;
constexpr uint8_t kOledAddr = 0x3C;
constexpr uint8_t kOledWidth = 128;
constexpr uint8_t kOledHeight = 64;

// --- IMU (optional) -------------------------------------------------
// MPU6050-class accel/gyro, sharing the OLED's I2C bus. 0x68 with AD0
// low, 0x69 with AD0 high. Absent hardware is detected at boot and the
// feature simply stays off -- the robot walks fine without it.
constexpr uint8_t kImuAddr = 0x68;

// --- Ultrasonic rangefinder (optional) ------------------------------
// HC-SR04 class. TRIG is a normal output.
//
// ECHO MUST BE LEVEL-SHIFTED. The HC-SR04 drives its echo line at 5V and
// the ESP32 is NOT 5V tolerant -- wiring it directly will damage the
// pin. Use a divider (e.g. 1k series with 2k to ground) or a proper
// level shifter.
//
// GPIO 35 is INPUT-ONLY on the ESP32, which suits an echo line exactly
// and keeps an output-capable pin free for something else.
constexpr uint8_t kRangeTrigPin = 27;
constexpr uint8_t kRangeEchoPin = 35;

// Stop before walking into something. Generous, because the sensor
// cone is wide and the robot is small.
constexpr uint16_t kObstacleStopMm = 150;

// --- Battery sense --------------------------------------------------
// MUST be an ADC1 pin: ADC2 is unusable while WiFi is active, which
// rules out most of the low GPIOs. Divider scales 2S Li-ion (~8.4V max)
// into the ESP32's ~3.3V range.
constexpr uint8_t kBatterySensePin = 34;

// --- Timing ---------------------------------------------------------
// The motion tick rate MUST equal the servo PWM frame rate. A servo only
// sees one pulse per frame, so updating faster than the frame rate just
// races the hardware; updating slower wastes resolution. Deriving one
// from the other keeps them from drifting apart when someone edits this.
constexpr uint32_t kPwmHz = 50;
constexpr uint32_t kTickMs = 1000 / kPwmHz;  // 20 ms

// MG90S pulse-width mapping for 0..180 servo degrees. Per-joint values
// live in the Calibration struct; these are the defaults it starts from.
constexpr int16_t kServoUsMin = 732;
constexpr int16_t kServoUsMax = 2929;

constexpr uint32_t kSerialBaud = 115200;

}  // namespace sesame
